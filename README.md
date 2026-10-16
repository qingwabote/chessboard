# chessboard

网格实现的 DOTS 2D 物理引擎，顶替 com.unity.physics：程序集 `Chessboard` / `Chessboard.Hybrid`，asmdef meta 继承官方 GUID，游戏侧按 GUID 引用零改动换装。

## 设计哲学

1. **API 贴着 dots physics 设计**。一切新功能对齐官方形状——形状从 `../physics-reference`（官方 1.4.5 源码副本）抄，行为按 chessboard 语义实现。同 API 不同后端是官方认可的平替模式（Havok 先例），不承诺数值逐位对齐。不自造原生形状。

2. **消费面驱动**。只实现真实消费者用到的接口；签名全集是无数消费面并集的极限，永远不是目标本身。新消费者出现→按其消费面增量扩。

3. **stateless 红线**。引擎是每步对当前 ECS 状态的纯函数：跨步缓存（以实体为键的稳定字段缓存）禁用。修正常等瞬态结构每步重建、用完即弃。

4. **web 是性能准绳**。目标平台微信小游戏：无 Burst、单线程。性能以云测实测为准（`PhaseTiming` 分解 + PhysicsStats 屏显），每实体 managed 组件调用是首敌（管线走 chunk 迭代）。

5. **行为等价，不是行为克隆**。验收 = PhysicsTest.unity 谓词面板（15 条）全绿 + 对局无回归；与官方的已知偏离（单遍位置投影、无速度求解器、事件真实重叠才发）是取舍，不是缺陷，修订需重新拍板。

6. **回退通道保持活性**。API 同形使 manifest 一行即可换回官方包——chessboard 未经受长期对局验证期间，这条退路不许焊死。

## 明确不做

- Joint / Mesh / Terrain / 多物理世界 / PhysicsShapeAuthoring——本项目零需求面
- 速度求解器（摩擦锥 / Baumgarte / 冲量迭代）——与单遍位置投影的 web 性能取舍冲突，需要时重新拍板
- 行为逐位对齐官方——不可达目标，不做承诺

## 参照物

- 官方源码副本（形状字典）：`C:\Users\logan\Documents\Unity\physics-reference`
- 正确性门禁：`survivor/Assets/Scenes/PhysicsTest/`（PhysicsTest.unity + PhysicsTestPanel 15 条谓词 + chessboard-test-scenes.md 底账）
- 官方语义对照与消费面底账：survivor 记忆 `project-chessboard-grid-physics-rebuild`
