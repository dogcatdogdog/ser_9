//! HTTP 服务层 (W7) — R6.2 调研结论定案
//!
//! 职责: 解析→调用→错误映射。solver 纯函数不接触本层。
//! 无共享状态 → 不需要 `State`, 保持无状态便于 oneshot 测试。
//! 错误体: 顶层 JSON `{"code": ..., "message": ...}` (A3_SCHEMA.md §2.2)。

use axum::extract::rejection::JsonRejection;
use axum::extract::Json;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::post;
use axum::Router;

use crate::dto::{ApiError, Defaults, MultiStopReq, RoutePlanResp};
use crate::solver::plan_multistop;

/// 服务端默认配置 (MVP: 常量, 后续可接环境变量)
const DEFAULT_MAX_ITERATIONS: usize = 20;
const DEFAULT_TIME_LIMIT_SECS: f64 = 5.0;
const DEFAULT_SEED: u64 = 42;

/// 构建无状态 Router — POST /plan (A3_SCHEMA.md §3)
pub fn app() -> Router {
    Router::new().route("/plan", post(plan_handler))
}

/// POST /plan 处理器: 解析 → 调用纯函数 → 序列化
///
/// serde 解析失败 (语法/缺字段/类型错误) 统一映射为 400 BAD_REQUEST
/// (R6.2 定案; axum JsonRejection 默认 422, 与契约不一致需显式映射)
async fn plan_handler(
    payload: Result<Json<MultiStopReq>, JsonRejection>,
) -> Result<Json<RoutePlanResp>, ApiError> {
    let Json(req) = payload.map_err(|rej| ApiError::bad_request(rej.body_text()))?;
    let cfg = Defaults {
        max_iterations: DEFAULT_MAX_ITERATIONS,
        time_limit_secs: DEFAULT_TIME_LIMIT_SECS,
        seed: DEFAULT_SEED,
    };
    let plan = plan_multistop(&req, &cfg)?;
    Ok(Json(plan))
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
    use axum::http::{Request, StatusCode};
    use http_body_util::BodyExt;
    use tower::ServiceExt;

    /// oneshot 直接调用 Router (不起端口) — 正常请求 → 200
    #[tokio::test]
    async fn plan_returns_200() {
        let body = r#"{
            "targets": [
                {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0},
                {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 5.0}
            ],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                       "alpha": 0.1, "beta": 0.005}
        }"#;
        let response = app()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(body))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let plan: RoutePlanResp = serde_json::from_slice(
            &response.into_body().collect().await.unwrap().to_bytes(),
        )
        .unwrap();
        assert!(plan.feasible);
        assert_eq!(plan.sequence.len(), 2);
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
        let response = app()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(body))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        let err: ApiError = serde_json::from_slice(&body).unwrap();
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("empty"));
    }

    /// 非法 JSON (语法错误) → 400 (serde 解析失败)
    #[tokio::test]
    async fn plan_malformed_json_returns_400() {
        let response = app()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"targets": [bad json"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    /// 缺字段 JSON (语义错误) → 400 (与语法错误同码, R6.2 定案)
    #[tokio::test]
    async fn plan_missing_field_json_returns_400() {
        let response = app()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"targets": []}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = response.into_body().collect().await.unwrap().to_bytes();
        let err: ApiError = serde_json::from_slice(&body).unwrap();
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("missing field"));
    }

    /// 不可行请求 (载重超限) → 200 + feasible=false (与 Python 一致:
    /// 不可行是合法求解结果, 而非错误)
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
        let response = app()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/plan")
                    .header("content-type", "application/json")
                    .body(Body::from(body))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let plan: RoutePlanResp = serde_json::from_slice(
            &response.into_body().collect().await.unwrap().to_bytes(),
        )
        .unwrap();
        assert!(!plan.feasible);
        assert!(plan.warnings.iter().any(|w| w.contains("exceeds drone capacity")));
    }
}
