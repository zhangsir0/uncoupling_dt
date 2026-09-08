# BehaviorTree.CPP 节点类型详解

> 基于 `uncoupling_robot_bt` 行为树的实际使用场景，覆盖 BehaviorTree.CPP v4 全部节点类型。

---

## 一、组合节点 (Composite) — 控制子节点的执行顺序

组合节点有多个子节点，按特定规则依次或并行执行。

### 1. Sequence → (序列)

```
子节点依次执行，全部 SUCCESS 才算 SUCCESS。
任一子节点 FAILURE → 立即中止，返回 FAILURE。
任一子节点 RUNNING → 停在当前位置，返回 RUNNING（下次 tick 从断点继续）。
```

**示例** (`config/uncoupling_robot_bt_dual_mode.xml` S1_IDLE):

```xml
<Sequence name="idle_standby">
  <Action ID="ApplyParkingBrake"/>     <!-- ① 先驻车 -->
  <Condition ID="IsBatteryNormal"/>    <!-- ② 检查电池 -->
  <Action ID="WaitForCIPSTask"/>       <!-- ③ 等待任务 -->
</Sequence>
```

顺序执行：驻车 → 检查电池 → 等待任务。任何一步失败，后续不再执行。

### 2. Fallback ? (选择器 / 优先级)

```
子节点依次尝试，任一 SUCCESS → 立即返回 SUCCESS。
全部 FAILURE → 返回 FAILURE。
任一 RUNNING → 停在当前位置，返回 RUNNING。
```

**示例** (`work_mode_dispatcher`):

```xml
<Fallback name="work_mode_dispatcher">
  <Sequence name="static_mode_flow">   <!-- ① 先试静态 -->
    <Condition ID="IsStaticMode"/>     <!-- work_mode==0? -->
    ...
  </Sequence>
  <Sequence name="dynamic_mode_flow">  <!-- ② 静态不满足→试动态 -->
    <Condition ID="IsDynamicMode"/>    <!-- work_mode==1? -->
    ...
  </Sequence>
</Fallback>
```

"先试这个，不行再试那个"。`IsStaticMode` FAILURE → 自动 fallback 到 `IsDynamicMode`。

### 3. ReactiveFallback ⚡ (响应式选择器)

```
与 Fallback 类似，但每个 tick 都从头重新检查所有子节点。
关键区别: 如果当前执行的子节点之前有一个更高优先级的条件变为 SUCCESS，
          会 halt（中断）当前子节点，转而执行高优先级分支。
```

**示例** (全局 E_STOP):

```xml
<ReactiveFallback name="global_estop_monitor">
  <Sequence name="estop_global_handler">  <!-- ① 急停分支 (高优先级) -->
    <Fallback name="estop_triggers">...   <!-- 检查急停条件 -->
    ...
  </Sequence>
  <Sequence name="main_flow">             <!-- ② 主流程 (低优先级) -->
    ...
  </Sequence>
</ReactiveFallback>
```

每个 tick 先检查急停条件。如果主流程正在 RUNNING 但急停突然触发 → **抢占式中断**主流程，立即执行急停。

**Fallback vs ReactiveFallback 对比**：

| | Fallback | ReactiveFallback |
|---|---|---|
| 子节点 RUNNING 时 | 继续 tick 当前子节点 | **每个 tick 重头检查** |
| 高优先级条件恢复 | 不会自动切换 | **自动抢占切换** |
| 适用场景 | 一次性选择（模式分支） | 持续监控（急停/看门狗） |

### 4. Parallel (并行)

```
所有子节点同时执行。
通过 success_threshold / failure_threshold 控制成功/失败条件。
```

---

## 二、装饰器节点 (Decorator) — 包装单个子节点，修改其返回值

### 1. Inverter ! (取反)

| 子节点返回 | Inverter 返回 |
|---|---|
| SUCCESS | FAILURE |
| FAILURE | SUCCESS |
| RUNNING | RUNNING (透传) |

**示例**:

```xml
<Inverter>
  <Condition ID="IsMagneticGuideOnline"/>
</Inverter>
```

| IsMagneticGuideOnline | Inverter 输出 | 含义 |
|---|---|---|
| SUCCESS (在线) | FAILURE | 不触发急停 |
| FAILURE (脱线) | **SUCCESS** | 触发急停! |

### 2. ForceSuccess ✓ / ForceFailure ✗

| 装饰器 | SUCCESS→? | FAILURE→? | RUNNING→? |
|--------|-----------|-----------|-----------|
| ForceSuccess | SUCCESS | **SUCCESS** | RUNNING |
| ForceFailure | **FAILURE** | FAILURE | RUNNING |

**ForceSuccess 作用**：KRUFF 退出返回 FAILURE，ForceSuccess 将其转为 SUCCESS，让 `main_flow` 正常收尾。

**ForceFailure 作用**：Sequence 末尾故意把成功扭成失败，迫使 KRUFF 回到 S1 开始下一轮循环。

### 3. KeepRunningUntilFailure (KRUFF)

```
子节点 SUCCESS → 重新执行 (循环)
子节点 FAILURE → 退出，返回 FAILURE
子节点 RUNNING → 继续 tick，返回 RUNNING
```

**语义**："一直做，直到失败"。只要子节点成功，就重新再来一轮。

**示例** (外层大循环):

```xml
<KeepRunningUntilFailure>
  <Sequence name="IdleToMissionLoop">
    <SubTree ID="S1_IDLE"/>
    <SubTree ID="S2_ARM_CHECK"/>
    <RetryUntilSuccessful>
      <Sequence name="SingleHookExecutionFlow">
        <SubTree ID="S3_NAV_WAIT_AREA"/>
        <SubTree ID="S4_TARGET_POSITIONING"/>
        ...
      </Sequence>
    </RetryUntilSuccessful>
    <SubTree ID="S12_RETURN_HOME"/>
    <ForceFailure>
      <AlwaysSuccess/>
    </ForceFailure>
  </Sequence>
</KeepRunningUntilFailure>
```

流转逻辑：

```
S1→S2→[内层循环摘钩]→S12→ForceFailure(扭成FAILURE)→Sequence返回FAILURE
→ KRUFF收到FAILURE→退出循环→返回FAILURE
→ 外层ForceSuccess转SUCCESS→任务完成
```

### 4. RetryUntilSuccessful

```
子节点 FAILURE → 重新执行 (重试)
子节点 SUCCESS → 退出，返回 SUCCESS
子节点 RUNNING → 继续 tick，返回 RUNNING
```

**语义**："一直试，直到成功"。失败就重来，成功就退出。

**与 KRUFF 的对比**：

| | KeepRunningUntilFailure | RetryUntilSuccessful |
|---|---|---|
| 循环条件 | SUCCESS→继续 | FAILURE→重试 |
| 退出条件 | FAILURE→退出 | SUCCESS→退出 |
| 适用场景 | "一直做，直到失败" | "一直试，直到成功" |
| 树中位置 | 外层大循环 (S1→S12) | 内层单钩循环 (S3→S11) |

### 5. SubTree (子树)

```
将另一个 BehaviorTree 作为子节点嵌入。
_autoremap="true": 共享根黑板（跨子树读写同一份数据）。
```

---

## 三、叶节点 (Action / Condition) — 执行具体逻辑

叶节点没有子节点，是树的"叶子"，做实际工作。

### 1. SyncActionNode (同步动作)

```cpp
BT::NodeStatus tick() override;  // 一次调用，立即返回 SUCCESS 或 FAILURE
```

**适用场景**：瞬时操作 — 发送指令、设置变量、状态判定。

**示例**：
- `ApplyParkingBrake` — 发一条 CAN 驻车指令，立即返回
- `Script` — 设置黑板变量 `code="work_mode:=1"`
- `EmergencyBrake` — 发送急停 CAN 帧
- `SendCoSpeedArmCommand` — 发送 Modbus 指令到 PLC

### 2. StatefulActionNode (异步动作 / AsyncActionNode)

```cpp
BT::NodeStatus onStart() override;    // 首次进入时调用一次
BT::NodeStatus onRunning() override;  // 之后每次 tick 调用
void onHalted() override;             // 被中断时调用（做清理）
```

**适用场景**：跨多个 tick 的持续操作 — 等待事件、监控进度、执行长时间运动。

**生命周期**：

```
首次 tick → onStart()
              ├── SUCCESS → 节点完成 (罕见)
              ├── FAILURE → 节点失败
              └── RUNNING → 进入执行中状态
                              ↓
后续 tick → onRunning()
              ├── SUCCESS → 完成
              ├── FAILURE → 失败
              └── RUNNING → 继续等待下次 tick
              
被抢占时 → onHalted()  ← 清理资源、安全停机
```

**示例**：

| 节点 | onStart | onRunning | onHalted |
|------|---------|-----------|----------|
| `WaitForCIPSTask` | 记录开始时间 | 轮询 CIPS 报文缓冲区 | 无操作 |
| `MPCTrackApproach` | 设 `mpc_enable=true` | 读 `mpc_status`，CONVERGED→SUCCESS | 设 `mpc_enable=false`，切 P 档 |
| `NavigateToStaticTarget` | 设 `vel_enable=true` | 读 `vel_status`，CONVERGED→SUCCESS | 设 `vel_enable=false`，驻车 |
| `WaitForManualReset` | 无操作 | 读 `manual_reset`，true→SUCCESS | 无操作 |
| `NavigateToOrigin` | 发 D 档指令 | 磁导航循迹到原点 | 切 P 档停车 |
| `WaitForTargetGap` | 读 `target_id` | 雷达锁定目标间隙 | 无操作 |
| `LockTargetGap` | 读 `target_id`, `gap_count` | 比对是否到达目标间隙 | 无操作 |
| `WaitBatteryRecovery` | 记录开始时间 | 轮询 BMS SOC | 无操作 |

### 3. ConditionNode (条件节点)

```cpp
BT::NodeStatus tick() override;  // 只读判断，不产生副作用，立即返回
```

**规则**：Condition 节点应该是**纯函数**——只读黑板/话题数据做判断，不写数据、不控制硬件。

**示例**：
- `IsBatteryNormal` — 检查黑板 SOC > 20%
- `IsStaticMode` — 检查黑板 `work_mode == 0`
- `IsDynamicMode` — 检查黑板 `work_mode == 1`
- `IsTargetInRange` — 检查雷达目标在视野内
- `IsCollisionRisk` — 检查障碍物距离
- `IsMagneticGuideOnline` — 检查磁导航数据帧率
- `IsEStopButtonPressed` — 检查 I/O 模块急停按钮
- `IsHeartbeatLost` — 检查 CAN/Modbus 心跳
- `IsAtPositionLimit` — 检查编码器/限位开关
- `IsTrackingErrorWithin` — 检查 MPC 误差 < ε
- `IsStabilized` — 检查稳态条件
- `CheckRemainingTasks` — 检查 `remaining_tasks`
- `CheckCoSpeedArm` — 检查共速臂寄存器状态
- `CheckMechanicalArm` — 检查机械臂寄存器状态

---

## 四、汇总对照表

| 节点 | 类型 | 子节点数 | 核心语义 |
|------|------|---------|---------|
| Sequence → | Composite | 多个 | 全成功=成功，串联执行 |
| Fallback ? | Composite | 多个 | 一个成功=成功，优先级选择 |
| ReactiveFallback ⚡ | Composite | 多个 | Fallback + 每 tick 重头检查 + 抢占 |
| Inverter ! | Decorator | 1 | 取反 |
| ForceSuccess ✓ | Decorator | 1 | 强制成功 |
| ForceFailure ✗ | Decorator | 1 | 强制失败 |
| KRUFF ⟳ | Decorator | 1 | 成功→循环，失败→退出 |
| RetryUntilSuccessful ↻ | Decorator | 1 | 失败→重试，成功→退出 |
| SubTree | Decorator | 1 | 嵌入另一个 Tree |
| SyncAction | Leaf | 0 | 瞬时动作，立即返回 |
| StatefulAction | Leaf | 0 | 异步动作，onStart→onRunning→onHalted |
| Condition | Leaf | 0 | 只读判断，立即返回 |

---

## 五、记忆口诀

| 节点 | 口诀 |
|------|------|
| Sequence | "AND" — 全部成功才是成功 |
| Fallback | "OR" — 一个成功就是成功 |
| ReactiveFallback | "OR + 每 tick 重头检查" — 急停监控专用 |
| KRUFF | "SUCCESS→继续循环" — 正常就重复 |
| RetryUntilSuccessful | "FAILURE→继续重试" — 失败就重来 |
| Inverter | "取反" — 在线变不在线，正常变异常 |
| ForceSuccess | "无论如何都成功" — 兜底收尾 |
| ForceFailure | "无论如何都失败" — 触发重试/循环 |
| StatefulAction | "三态: 开始→执行中→清理" — 长时间操作专用 |
| Condition | "纯函数，只看不写" — 状态判断专用 |
