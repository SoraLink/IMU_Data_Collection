# 采集流程：IMU 与 8 相机视频的手工时间同步

针对每位被试 10–20 个动作、每个动作都需要独立标定和同步的场景。
相机之间的同步不在此处理，这里只管 IMU 侧。

## 每个 take 的结构

一个 take 是**一段连续不间断的录制**，内部依次包含四个阶段：

```
T-POSE  →  跳跃①  →  动作  →  跳跃②
```

- **T-POSE** 给这个 take 一份标定基准。逐 take 而不是逐 session 采集，
  所以传感器在动作间发生轻微位移也不会污染后续数据。
- **两次跳跃**把动作夹在中间。跳跃在两种信号里都无法被误认：自由落体让
  `|acc|` 掉向 0，落地则打出一个尖峰。首尾各一次意味着你既能在起点对齐，
  又能在终点复核——两端对不上就说明这个 take 中间有漂移。

两次跳跃**动作完全相同**，都是原地跳一下，只是承担的角色不同，所以界面上
用 ①② 编号而不是两个不同的名字。

阶段的起止是**粗略**的，这没关系：它们只用来圈定搜索范围。真正用于对齐的
是系统在每个跳跃窗口内自动找出的自由落体最低点和落地峰值，以 packet
counter 写进 `metadata.json`。

## 为什么对齐要用 counter 而不是时间戳

`counter` 是 MTw 的包计数器，由基站统一下发，**17 个传感器共享同一个
counter**，所以一个 counter 值在所有传感器上指向同一时刻。主机侧的到达时间
带有无线抖动和重传延迟，不能用。counter 是 16 位、到 65536 回绕，分析时记得
展开。

## 运行方式

采集器在虚拟机里跑（SDK 在那儿），它自带一个网页界面，用 Chrome 打开即可。
3D 渲染因此发生在宿主机的 GPU 上，而不是被模拟的客户机里——后者会慢到无法
使用。数据落盘发生在 SDK 回调里，**界面再慢也不会丢样本**。

**终端 1 —— 启动采集器（虚拟机内）**

```bash
cd vm && ./vssh 'cd ~ && python3 -u session_record.py --session session1'
```

**终端 2 —— 打通隧道**

```bash
ssh -p 2222 -i ~/.ssh/xsens_vm -L 9000:localhost:9000 -N xsens@127.0.0.1
```

**浏览器 —— 打开 `http://localhost:9000`**

采集器会先把 HTTP 服务开起来再去连硬件，所以网页可以提前打开，会显示
「等待传感器连接…」而不是报连接失败。页面在手机上同样可用，站在场地里
用手机点即可。

## 界面与操作

| 操作 | 作用 |
|---|---|
| 填「被试 ID」→ 设定 | 每位被试一次。切换会新建 session 目录、take 编号归零 |
| 填动作名 → `开始 take` | 或按 `n` 跳到输入框，回车即开始 |
| `空格` / 点按钮 | 开始下一个阶段（按钮上会写明是哪个阶段） |
| `r` | 丢弃当前 take 重录 |

**节奏的分工**：每个阶段「什么时候开始」由你点击决定，「什么时候结束」由
设定的时长决定。因为一旦阶段开始，你人在场地中间，够不着任何按钮。点击后
先走一段倒计时（屏幕上是超大数字，配逐秒提示音），倒计时归零才真正开始记录
该阶段——所以倒计时不会占用跳跃窗口。

每个阶段的倒计时默认 5 秒。右侧「节奏设置」里每个阶段的倒计时和时长
都能单独调——比如场地离电脑远，就把 T-pose 的倒计时加长。

## 一个 take 的完整操作

1. 填动作名，点 `开始 take`
2. 走进场地 —— 5 秒倒计时 —— 摆 T-pose 站稳 5 秒
3. 停在等待态。点 `开始跳跃 ①` —— 5 秒倒计时 —— 原地跳一下
4. 停在等待态。点 `开始动作` —— 5 秒倒计时 —— 执行动作
5. 停在等待态。点 `开始跳跃 ②` —— 5 秒倒计时 —— 再原地跳一下
6. 自动保存，回到待机

3D 骨架在 T-pose 采到之后会立正，这是标定成功的直观确认。右侧传感器表
掉线的会标红；鼠标悬停某一行会高亮 3D 骨架上对应的骨骼，戴传感器时用得上。

**如果右下角提示 `⚠ 信号弱`**，说明那段窗口里没找到可信的跳跃特征
（自由落体没低于 4 m/s²，或落地没超过 15 m/s²）。多半是没真跳、或者跳早了
落在窗口外。当场按 `r` 重录——比事后发现对不齐要省事得多。

## 产出结构

```
data/S01/20260916_session1/
  session.json                 ← 全 session 索引，含每个 take 的同步点摘要
  take01_jump_forward/
    pelvis.csv  sternum.csv  ... （17 个）
    metadata.json
  take02_squat/
  ...
```

`metadata.json` 里对齐相关的部分：

```json
"events": [
  {"event": "take_start",     "counter": 33670, "t_since_take_start": 0.00},
  {"event": "t_pose_start",   "counter": 33670, "t_since_take_start": 0.00},
  {"event": "t_pose_end",     "counter": 33969, "t_since_take_start": 4.97},
  {"event": "jump_in_start",  "counter": 33970, "t_since_take_start": 5.01},
  {"event": "jump_in_end",    "counter": 34187, "t_since_take_start": 8.60},
  {"event": "action_start",   "counter": 34187, "t_since_take_start": 8.60},
  {"event": "action_end",     "counter": 34524, "t_since_take_start": 14.23},
  {"event": "jump_out_start", "counter": 34524, "t_since_take_start": 14.23},
  {"event": "jump_out_end",   "counter": 34740, "t_since_take_start": 17.82}
],
"jump_sync": {
  "jump_in":  {"freefall_counter": 34097, "landing_counter": 34112, "detected": true},
  "jump_out": {"freefall_counter": 34601, "landing_counter": 34617, "detected": true}
},
"tpose_quat": {"pelvis": [0.9156, 0.0062, 0.0030, -0.4020], ...}
```

- **`jump_sync.*.freefall_counter`** 是最稳的对齐锚点：自由落体是一段持续
  十几毫秒的低谷，而不是单样本尖峰，抗噪声更好。
- **`landing_counter`** 是落地冲击峰值，在视频里最容易一眼找到，适合做
  人工复核。
- **`tpose_quat`** 是 T-pose 期间每个传感器的平均姿态（四元数）。CSV 里
  保持原始数据不动，标定量单独存放，后期可以随时重算或换用别的标定方式。
- **`heading_deg` / `view_offset_deg`** 只影响实时 3D 骨架的朝向，不动数据。

## 3D 骨架朝向（仅影响显示，不影响录制数据）

IMU 四元数的参考系是**磁北**，不是屏幕。渲染把每段旋转从磁北系转到屏幕系，
需要知道"哪边是屏幕"——而数据里没有这个信息。做法：

- **本 session 的第一个 take 定义"正前方"**：它的骨盆朝向作为参考，自己
  `heading=0`；后续每个 take 相对它自动转正（骨盆朝向之差）。所以**第一个
  take 录制时朝向要对**（面向你希望骨架朝的方向）。
- 换被试会重新佩戴，参考自动重置，下一个 take 重新定义正前方。
- 万一第一个 take 朝向也不理想，用界面上的手动偏移按钮整体转一下（整个
  session 保持）。
- 这套只影响骨架看起来朝哪，**CSV 里的原始四元数/加速度完全不受影响**，
  时序同步和后续分析用的是原始数据。

## 对齐的做法

1. 在视频里找到跳跃① 的那一帧（落地瞬间最好认）。
2. 用 `jump_sync.jump_in.landing_counter` 作为 IMU 侧的同一时刻。
3. 偏移量 = 视频时间 − `(counter − take_start_counter) / update_rate_hz`。
4. 用 `jump_out` 复核一遍。两端算出的偏移若差异明显，说明这个 take
   存在时钟漂移或丢包，需要单独处理。

## 加工后数据(可直接喂模型)

`postprocess.py` 把一个 take 的原始 CSV 转成 sparse-IMU 姿态模型(TransPose /
DIP 等)要的格式,输出到该 take 的 `processed/` 目录。**原始 CSV 不动**,这是
额外多给的一份。

```bash
python3 postprocess.py data/S01/20260916_session1/take01_squat      # 单个 take
python3 postprocess.py --session data/S01/20260916_session1         # 整个 session
```

产物:

- **`calibrated.npz`**(与模型无关,17 段全有):
  - `ori_smpl (T,17,3,3)` —— 每段标定到骨骼(SMPL)系的朝向(旋转矩阵)
  - `acc_free_smpl (T,17,3)` —— 去重力后的加速度,骨骼系
  - `gyr (T,17,3)` —— 角速度
  - `segment_order`(17 段顺序,pelvis 最后)、`counter_grid`(规整 60Hz 时间轴)、
    `smpl2imu` / `gravity` / `acc_scale`(所用标定)
- **`transpose_input.npy`** `(T,72) = [18 加速度 | 54 朝向]` —— TransPose live-demo
  的 6 传感器输入布局(根相对,自包含,无需外部统计文件)

处理步骤:选段 → 四元数转旋转矩阵 → T-pose 标定 sensor-to-bone → 解析法去重力
(`a·R − [0,0,9.81]`)→ 按 packet counter 对齐、**丢包缺口插值补齐** → 规整到
60Hz → 根相对归一化(喂模型这步本身朝向无关)。

**标定的已知近似**(见 `postprocess.py` 顶部注释):device2bone 用"T-pose 时骨骼
= 单位"的假设;smpl2imu 让骨盆 T-pose 朝向对齐 SMPL 前方。格式和物理量(单位、
坐标系、去重力、丢包、频率)是准的;但"某个预训练模型能否直接吃出好姿势"需要
拿它的权重实际跑才能确认,本脚本不保证这一点。
