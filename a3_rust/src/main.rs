//! A3 · 多目标访问路线规划 — HTTP 服务 (bin)
//!
//! W7: axum 服务 POST /plan on port 9204 (R6.2 定案)。
//! 核心纯函数在 lib (dto/energy/solver/heuristic), 本文件只做启动。

use axum::serve;
use tokio::net::TcpListener;

const PORT: u16 = 9204;

#[tokio::main]
async fn main() {
    let addr = format!("0.0.0.0:{PORT}");
    let listener = TcpListener::bind(&addr)
        .await
        .unwrap_or_else(|e| panic!("bind {addr} 失败: {e}"));
    println!("A3 solver listening on http://{addr} (POST /plan)");
    serve(listener, a3_rust::http::app())
        .await
        .expect("server error");
}
