//! A3 · 多目标访问路线规划 — 核心纯函数库 + HTTP 层
//!
//! W6: 骨架 (dto/energy/solver 空壳) + 交叉验证测试 (tests/cross_check.rs)
//! W7: 算法实现 (heuristic NN + VND) + axum 服务 (http.rs, main.rs 启动)

pub mod dto;
pub mod energy;
pub mod heuristic;
pub mod http;
pub mod solver;

#[cfg(test)]
mod heuristic_tests;
