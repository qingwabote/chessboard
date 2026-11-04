# chessboard

网格实现的 DOTS 2D 物理引擎，API 是 com.unity.physics 的子集，asmdef GUID 与之相同，便于迁移

## 设计哲学

- “API 是官方 com.unity.physics 的子集”只在签名上保持一致，实现上简单优先于一致，数量上按消费项目删减至最低

- **官方子集之外另有网格游标原语**（不与官方 API 同形，不参与换回官方包的兼容承诺）：`CreateBodyVisitor`/`BodyVisitor.Visit(Range2)`（任意格矩形内去重迭代 body 索引，结构层、与形状无关；同一 BodyVisitor 的多次 Visit 共享一个去重代，须连续耗尽）与公开读数 `Grid`/`CellSize`；半径/top-K 等策略语义归消费项目（survivor 的 `RvoNearest`）。编辑器下同一时刻只允许一个开放的 BodyVisitor，开放期间其它碰撞查询在入口、二开与 Dispose 后调用均以编辑器断言报错（release 全部剥净）

- stateless，与官方一致

- **影子语义**。碰撞真身 = 棋子在 xz 平面的正投影（**影子**）。三根旋转轴全部以投影形式参与碰撞（躺倒的高件影子变长、倒在哪挡到哪），没有任何一根把碰撞带回 3D；y 回归纯渲染维：遮挡梯子、浮动动画、击飞抛物线零碰撞后果，碰撞只认脚印，永不反向逼迫视觉改 y；接触法线恒水平（y=0），纯 2D，无任何 y 闸门。查询同域：OverlapAabb 输入的 y 分量不参与命中判定。

- **投机接触**。间距小于 `相对速度×Δt + PhysicsStep.CollisionTolerance` 的配对在穿透发生前即产生接触（预防常开，无开关）：一遍法向速度解防止本步内按当前速度将要发生的穿透（解后法向相对速度封顶：动态对 `PhysicsStep.MaxDynamicDepenetrationVelocity`、动静对 `MaxStaticDepenetrationVelocity` 默认 +∞ 不限，旋钮 ≤0 回退默认），碰撞事件按需预发。目的 = 正常玩法零重叠，不是通用求解器。

## 参照物

- 官方源码副本（形状字典）：`C:\Users\logan\Documents\Unity\physics-reference`
- 正确性门禁：`survivor/Assets/Scenes/PhysicsTest.unity`（PhysicsTestPanel 谓词）
- 官方语义对照与消费面底账：survivor 记忆 `project-chessboard-grid-physics-rebuild`
