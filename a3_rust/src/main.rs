//! A3 · 多目标访问路线规划 — HTTP 服务 (bin)
//!
//! W7: axum 服务 POST /plan (R6.2 定案)。
//! W8 硬化: tracing 日志 + env 配置 (A3_PORT / A3_MAX_ITERATIONS /
//! A3_TIME_LIMIT_SECS), 默认值即文档值。
//! 核心纯函数在 lib, 本文件只做启动 + 配置装配。

use axum::serve;
use tokio::net::TcpListener;

use a3_rust::http::{app_with, ServiceConfig};

/// 默认监听端口
const DEFAULT_PORT: u16 = 9204;

/// env 读取辅助: 缺失/解析失败 → 默认值
fn env_or<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    let port: u16 = env_or("A3_PORT", DEFAULT_PORT);
    let cfg = ServiceConfig {
        max_iterations: env_or("A3_MAX_ITERATIONS", 20usize),
        time_limit_secs: env_or("A3_TIME_LIMIT_SECS", 5.0f64),
    };

    let addr = format!("0.0.0.0:{port}");
    let listener = TcpListener::bind(&addr)
        .await
        .unwrap_or_else(|e| panic!("bind {addr} 失败: {e}"));
    tracing::info!(%addr, "A3 solver listening (POST /plan, GET /healthz)");
    serve(listener, app_with(cfg)).await.expect("server error");
}
