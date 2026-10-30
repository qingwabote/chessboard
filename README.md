# chessboard

网格实现的 DOTS 2D 物理引擎，API 是 com.unity.physics 的子集，asmdef GUID 与之相同，便于迁移

## 设计哲学

- “API 是官方 com.unity.physics 的子集”只在签名上保持一致，实现上简单优先于一致，数量上按消费项目删减至最低

- stateless，与官方一致

- **影子语义**。碰撞真身 = 棋子在 xz 平面的正投影（**影子**）。三根旋转轴全部以投影形式参与碰撞（躺倒的高件影子变长、倒在哪挡到哪），没有任何一根把碰撞带回 3D；y 回归纯渲染维：遮挡梯子、浮动动画、击飞抛物线零碰撞后果，碰撞只认脚印，永不反向逼迫视觉改 y；接触法线恒水平（y=0），纯 2D，无任何 y 闸门。

- **投机接触**。间距小于 `PhysicsStep.SpeculativeMargin`（默认 0.1 m）的配对在穿透发生前即产生接触：一遍法向速度解防止本步内按当前速度将要发生的穿透（每步法向速度修正量封顶 `MaxDynamicDepenetrationVelocity`），碰撞事件按需预发。目的 = 正常玩法零重叠（与官方投机接触同向），不是通用求解器。

## 参照物

- 官方源码副本（形状字典）：`C:\Users\logan\Documents\Unity\physics-reference`
- 正确性门禁：`survivor/Assets/Scenes/PhysicsTest.unity`（PhysicsTestPanel 谓词）
- 官方语义对照与消费面底账：survivor 记忆 `project-chessboard-grid-physics-rebuild`
