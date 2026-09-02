//! HTTP 服务层 (W7, W8 硬化) — R6.2 调研结论定案
//!
//! 职责: 解析→调用→错误映射。solver 纯函数不接触本层。
//! 错误体: 顶层 JSON `{"code": ..., "message": ...}` (A3_SCHEMA.md §2.2)。
//!
//! W8 硬化 (服务工程化):
//!   P0 — panic 兜底: middleware 以 tokio task 隔离 handler, panic → 500 JSON
//!   P1 — 并发:      plan_multistop 在 `spawn_blocking` 执行 (CPU 密集隔离),
//!                   请求级超时兜底 (tokio::time::timeout)
//!   P2 — 可运维:    GET /healthz + tracing 请求日志 + ServiceConfig (env 可配)

use std::time::{Duration, Instant};

use axum::extract::rejection::JsonRejection;
use axum::extract::{Json, State};
use axum::http::{Request, StatusCode};
use axum::middleware::{self, Next};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::Router;

use crate::dto::{ApiError, Defaults, MultiStopReq, RoutePlanResp};
use crate::solver::plan_multistop;

/// 服务配置 (W8: State 注入, env 可覆盖, 见 main.rs)
#[derive(Debug, Clone)]
pub struct ServiceConfig {
    /// VND 外层迭代上限
    pub max_iterations: usize,
    /// 求解时限 (秒); <= 0 表示不限
    pub time_limit_secs: f64,
}

impl Default for ServiceConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            time_limit_secs: 5.0,
        }
    }
}

/// 请求级超时兜底 (秒) — 远大于正常求解 (<150ms), 仅防失控
const REQUEST_TIMEOUT_SECS: u64 = 10;
/// 随机种子 (算法确定性, 保留字段)
const DEFAULT_SEED: u64 = 42;

/// 构建 Router (默认配置) — POST /plan + GET /healthz
pub fn app() -> Router {
    app_with(ServiceConfig::default())
}

/// 构建 Router (指定配置) — 配置经 State 注入 handler 与中间件
pub fn app_with(cfg: ServiceConfig) -> Router {
    Router::new()
        .route("/plan", post(plan_handler))
        .route("/healthz", get(healthz))
        .layer(middleware::from_fn_with_state(cfg.clone(), observe_layer))
        .with_state(cfg)
}

/// 健康检查 — 探活基本前提 (P2)
async fn healthz() -> &'static str {
    "ok"
}

/// POST /plan 处理器: 解析 → spawn_blocking 调纯函数 → 序列化
///
/// 错误语义 (契约):
///   - serde 解析失败 (语法/缺字段) → 400 BAD_REQUEST (axum 默认 422, 显式映射)
///   - 参数非法 (空/超限/参数域) → 400 BAD_REQUEST (solver 校验传播)
///   - 不可行求解 → 200 + feasible=false (与 Python 对齐, 非错误)
async fn plan_handler(
    State(cfg): State<ServiceConfig>,
    payload: Result<Json<MultiStopReq>, JsonRejection>,
) -> Result<Json<RoutePlanResp>, ApiError> {
    let Json(req) = payload.map_err(|rej| ApiError::bad_request(rej.body_text()))?;
    let defaults = Defaults {
        max_iterations: cfg.max_iterations,
        time_limit_secs: cfg.time_limit_secs,
        seed: DEFAULT_SEED,
    };

    // P1: CPU 密集求解移出 async worker (spawn_blocking 阻塞线程池),
    // 外层 tokio::time::timeout 兜底防失控 (blocking 任务无法真正取消,
    // 超时后立即返回错误, 线程池任务自然跑完)
    let handle = tokio::time::timeout(
        Duration::from_secs(REQUEST_TIMEOUT_SECS),
        tokio::task::spawn_blocking(move || plan_multistop(&req, &defaults)),
    )
    .await
    .map_err(|_| ApiError::internal("solver exceeded request timeout"))?;

    let plan = handle
        .map_err(|e| ApiError::internal(format!("solver task panic: {e}")))??;

    Ok(Json(plan))
}

/// 观测中间件 (P0 + P2):
///   1. 以 tokio task 隔离 handler — panic 捕获为 500 JSON (而非断连)
///   2. tracing 请求日志: method / path / status / 耗时
async fn observe_layer(
    State(_cfg): State<ServiceConfig>,
    request: Request<axum::body::Body>,
    next: Next,
) -> Response {
    // from_fn_with_state 需同型 State; 配置当前无扩展用途, 保留占位
    let start = Instant::now();
    let method = request.method().clone();
    let path = request.uri().path().to_string();

    // handler panic → JoinError → 500 JSON (不中断连接)
    let response = match tokio::task::spawn(async move { next.run(request).await }).await {
        Ok(resp) => resp,
        Err(_) => ApiError::internal("handler panic").into_response(),
    };

    tracing::info!(
        method = %method,
        path = %path,
        status = response.status().as_u16(),
        elapsed_ms = start.elapsed().as_millis() as u64,
        "request"
    );
    response
}

/// ApiError → HTTP 响应 (R6.2 定案):
///   BAD_REQUEST → 400, INFEASIBLE → 422, INTERNAL → 500
impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let status = match self.code.as_str() {
            "BAD_REQUEST" => StatusCode::BAD_REQUEST,
            "INFEASIBLE" => StatusCode::UNPROCESSABLE_ENTITY,
            _ => StatusCode::INTERNAL_SERVER_ERROR,
        };
        (status, Json(self)).into_response()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::{Method, Request};
    use http_body_util::BodyExt;
    use tower::ServiceExt;

    fn plan_body() -> &'static str {
        r#"{
            "targets": [
                {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0},
                {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 5.0},
                {"id": "c3", "location": {"x": 100.0, "y": 100.0}, "demand": 5.0}
            ],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                       "alpha": 0.1, "beta": 0.005}
        }"#
    }

    async fn post_plan(router: Router, body: &str) -> Response {
        router
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(body.to_string()))
                    .unwrap(),
            )
            .await
            .unwrap()
    }

    /// 正常请求 → 200 (spawn_blocking + State 路径)
    #[tokio::test]
    async fn plan_returns_200() {
        let response = post_plan(app(), plan_body()).await;
        assert_eq!(response.status(), StatusCode::OK);
        let plan: RoutePlanResp = serde_json::from_slice(
            &response.into_body().collect().await.unwrap().to_bytes(),
        )
        .unwrap();
        assert!(plan.feasible);
        assert_eq!(plan.sequence.len(), 3);
    }

    /// 健康检查 → 200 "ok"
    #[tokio::test]
    async fn healthz_returns_200() {
        let response = app()
            .oneshot(
                Request::builder()
                    .method(Method::GET)
                    .uri("/healthz")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        assert_eq!(&body[..], b"ok");
    }

    /// app_with: 自定义配置经 State 生效 (请求仍正常返回)
    #[tokio::test]
    async fn app_with_custom_config_serves() {
        let cfg = ServiceConfig {
            max_iterations: 1,
            time_limit_secs: 1.0,
        };
        let response = post_plan(app_with(cfg), plan_body()).await;
        assert_eq!(response.status(), StatusCode::OK);
        let plan: RoutePlanResp = serde_json::from_slice(
            &response.into_body().collect().await.unwrap().to_bytes(),
        )
        .unwrap();
        assert!(plan.feasible);
        assert_eq!(plan.sequence.len(), 3);
    }

    /// 空 targets → 400 BAD_REQUEST (solver 校验传播)
    #[tokio::test]
    async fn plan_empty_targets_returns_400() {
        let body = r#"{
            "targets": [],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                       "alpha": 0.1, "beta": 0.005}
        }"#;
        let response = post_plan(app(), body).await;
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        let err: ApiError = serde_json::from_slice(&body).unwrap();
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("empty"));
    }

    /// 非法参数域 (alpha=0) → 400 BAD_REQUEST (W8 校验, 防 panic)
    #[tokio::test]
    async fn plan_alpha_zero_returns_400() {
        let body = r#"{
            "targets": [
                {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0}
            ],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                       "alpha": 0.0, "beta": 0.005}
        }"#;
        let response = post_plan(app(), body).await;
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        let err: ApiError = serde_json::from_slice(&body).unwrap();
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("alpha"));
    }

    /// 非法 JSON (语法错误) → 400
    #[tokio::test]
    async fn plan_malformed_json_returns_400() {
        let response = post_plan(app(), r#"{"targets": [bad json"#).await;
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    /// 缺字段 JSON (语义错误) → 400 (与语法错误同码)
    #[tokio::test]
    async fn plan_missing_field_json_returns_400() {
        let response = post_plan(app(), r#"{"targets": []}"#).await;
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        let err: ApiError = serde_json::from_slice(&body).unwrap();
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("missing field"));
    }

    /// 不可行请求 (载重超限) → 200 + feasible=false (非错误)
    #[tokio::test]
    async fn plan_overload_returns_200_with_infeasible() {
        let body = r#"{
            "targets": [
                {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 8.0},
                {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 8.0}
            ],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {"payload_capacity": 10.0, "battery_capacity": 5000.0,
                       "alpha": 0.1, "beta": 0.005}
        }"#;
        let response = post_plan(app(), body).await;
        assert_eq!(response.status(), StatusCode::OK);
        let plan: RoutePlanResp = serde_json::from_slice(
            &response.into_body().collect().await.unwrap().to_bytes(),
        )
        .unwrap();
        assert!(!plan.feasible);
        assert!(plan.warnings.iter().any(|w| w.contains("exceeds drone capacity")));
    }
}
