// ============================================================
//  camera_hdmi.v
//  OV5640 DVP 采集 -> 灰度化 / Sobel 边缘 -> 640x480@60 HDMI(TMDS)
//
//  板卡 : 奥唯思 VF-Ti60F225 V1.5 (Efinix Titanium Ti60F225, 225-FBGA)
//  工具 : Efinity 2026.1.132.4.5  /  timing model C4
//
//  ------------------------------------------------------------------
//  端口列表是从 PT 生成的 outflow/camera_hdmi_template.v 逐字抄来的，
//  不要随手改名 —— 名字一变，peri.xml / lpf 就全对不上了。
//  ------------------------------------------------------------------
//
//  时钟体系（全部由 hdmi_pll 产生，参考 clk24m = 24 MHz 晶振, ball C5）
//      clk_sys       =  96.0  MHz   (M4/N1/O2, DIV0=21)   —— 心跳/慢计数
//      clk_pixel     =  25.2  MHz   (DIV1=80)             —— 显示像素时钟
//      clk_pixel_10x = 126.0 MHz    (DIV2=16, phase 22.5) —— LVDS 串行时钟
//
//      ★ lvds_rule_tx_clock 要求 fast clk 必须正好是 slow clk 的 5.0 倍。
//        126 / 25.2 = 5.0 ✓  正因为这条硬约束，分辨率只能选
//        640x480@60 (25.2 MHz)，标准的 720p 在 Ti60 的 PLL 分频范围内
//        凑不出 5 倍关系。
//
//      cmos_pclk : OV5640 输出的像素时钟（≈96 MHz 上限），独立异步域
//
//  数据通路
//      RGB565 ──> Y8 = (77R + 150G + 29B) >> 8
//                   │
//                   ├─ 3 个 640x8 行缓存（角色轮转）──> 3x3 窗口
//                   │                                     │
//                   │                                  Sobel |Gx|+|Gy|
//                   │                                     │
//                   └──────────> 模式选择 <───────────────┘
//                                   │
//                  ★ 2x2 降采样（取偶行偶列，每格恰写一次）
//                                   │
//                  （FB_DW<8 时才）4x4 抖动 -> 截到 FB_DW 位
//                                   │
//                ★ 帧缓存 320x240 @8bpp x 2 块乒乓（约 150 块 RAM10 / 256）
//                                   │  跨时钟
//                     显示读口 (clk_pixel)，只在帧起始换 bank
//                                   │
//                  ★ 整数最近邻 2x 放大（dcol=dx[9:1], drow=dy[9:1]）
//                                   │
//                      低位复制高位 -> 3 x tmds_channel --> LVDS_TX
//                                   │
//                      + 顶栏 12 行：左端 8px 恒白(显示探针) + 对焦条
//                      + 顶栏右端 12x12：摄像头帧心跳块（≈0.5 s 闪一次）
//                      + 底栏 12 行：纯显示域 4x4 棋盘格(二分探针)
//
//  正常显示档由参数 DISPLAY_MODE 固定，不再自动轮换（避免棋盘格 /
//  纯 Sobel 不断打断真实摄像头画面）。上板默认 = 0 纯灰度。
//  传感器内置彩条仍会短暂出现（开机 2.8 s 起、之后每 22.4 s 亮 2.8 s），
//  作为"数字通路全好"的健康自检。
//
//  ★★ 两条"不经过帧缓存"的硬探针（第 8 轮加，专治"全黑屏无法二分"）
//     1) 顶栏最左 8 像素**恒定白**：只由显示时序 + TMDS + LVDS 构成。
//        看得到它 ⇒ 显示通路从 PLL 到 HDMI 全活着。
//     2) 底部 12 行**纯显示域 4x4 棋盘格**（CHK_BAND）：只用 hcnt/vcnt 生成，
//        不碰帧缓存、不碰摄像头、不碰 CDC。
//        看得到棋盘格 ⇒ 显示 + TMDS + HDMI 全对，问题必在帧缓存/摄像头一侧；
//        看不到棋盘格 ⇒ 问题在显示时钟(PLL) / LVDS 一侧。
//     ★ 教训：**不要再加"帧缓存没填满就先出黑屏"这类门控**。
//       它会把"摄像头没出帧""帧缓存没写进去""显示通路死了"三种故障
//       全都表现成同一块纯黑屏，等于亲手把诊断信息扔掉。
//       BRAM 配置后初值本来就是 0，上电初期就是黑的，不需要额外遮。
//
//  ★ 屏幕顶部有一条"对焦指示条"（12 行高）：
//      白条长度 = 本帧整幅画面的平均边缘能量（Sobel 幅度累加 / 像素数）。
//      镜头越接近合焦，画面高频细节越多 => 白条越长。
//      于是"调焦"这件事从"肉眼看糊不糊"变成了"把白条顶到最长"，
//      不需要电脑、不需要来回发视频。
//      左端 8 像素恒白（显示探针）；中灰底 0x55 = 量程（640 像素满格）；
//      mode 3 自检档不画，免得盖住棋盘格。
//
//  ★ 无撕裂（双缓存乒乓）—— 第 7 轮的核心改动
//     单缓存下无论怎么对齐读指针都不可能稳：OV5640 的自动曝光会动态改
//     帧长（VTS），摄像头帧率一变，"读指针领先写指针多少"就跟着变，
//     撕裂线就会满屏乱漂（实拍已验证）。现在改成**两块缓存乒乓**：
//       写口只在摄像头帧起始把自己写满的那一块交出去、换写另一块；
//       读口收到后必须等到**显示帧起始**才真正换 bank。
//     于是每一帧显示的像素 100% 来自同一块缓存 ⇒ 撕裂在结构上不可能出现。
//     ★ BRAM 账本（每块 RAM10 有效 8192 bit，共 256 块 = 2 097 152 bit）：
//         "总位数 / RAM 总块数 <= 8192" 是判断存储到底装没装下的唯一硬判据。
//         ★★ 第 8j 轮的结论：**死磕全分辨率是错的**。
//            640x480 双缓存 => 每像素只有 3.41 bit
//              5bpp x 2 = 375 块 ✗   4bpp x 2 = 300 块 ✗
//              3bpp x 2 = 225 块 ✓（但只有 8 级灰度，抖动幅度 12.5%，
//                                   实拍就是满屏麻点 —— 见 _scratch/shot8/）
//            320x240 双缓存 => 每像素 13.6 bit，8bpp 只用 150 块，空出 106 块。
//        所以本工程最终选 **320x240 双缓存 + 8bpp**：拿一半分辨率
//        换回 256 级灰度、零抖动、零麻点，观感反而好得多
//        （镜头本来就失焦，半分辨率几乎看不出差别）。
//        同一路线在彩色姊妹工程 camera_color 里也是这么走的。
//
//  ★ 其它已知取舍
//    1) Sobel 的 3x3 窗口中心固定在"当前列 - 2"，所以帧的第 0、1、639
//       这几列不会被写入，会保留上一帧的值。屏幕上只有 1 个像素宽，
//       肉眼不可见。
//    2) 模组 CTL1 = PWDN（高电平掉电）、CTL3 = STB，三个 CTL 必须
//       全部给 0。这是上一次 bring-up 踩过的坑：CTL1 给 1 会导致
//       整块板 LED 全灭。
// ============================================================

module camera_hdmi #(
    parameter IMG_W     = 640,
    parameter IMG_H     = 480,
    // 叠加模式下"算不算边缘"的阈值（sobel8 的 0..255 尺度）
    // ★ 第 25aj 轮：96 → 48。用户视频反馈分屏右半边缘太稀疏（只有强边界可见），
    //   自然场景里多数边缘阶跃在 30~80，被 96 全滤掉了。降到 48 让边缘更丰富。
    // ★ 第 25an 轮：sobel8 去 /4 压缩后，单一阈值 48 落在传感器噪声带（20~50），
    //   背景平坦区噪声被点亮且逐帧闪烁。改为双阈值磁滞：SOBEL_HI 定强边缘，
    //   SOBEL_THR 定弱边缘（需邻域生长才保留），杀死孤立闪烁白点。
    // ★ 第 25ao 轮：HI=96 太苛刻，面部弱边缘线起点（周围无强边缘）被 LO 档拒掉，
    //   "要扭动才能看到线条"。降到 HI=64 让弱边缘线自己立住；噪点靠"时域一致性
    //   AND"（下节）来杀，不再只靠磁滞种子。
    parameter SOBEL_THR = 8'd48,
    parameter SOBEL_HI  = 8'd64,
    // 0=纯灰度，1=纯 Sobel，2=灰度+白边，3=2x2 棋盘。
    // ★ 上板默认用 0（纯灰度）：判断"清晰度 + 稳定度"要看摄像头**原始画面**，
    //   纯灰度最干净。mode 2 会把超过阈值的像素刷成白色，而 5b 节的自动对比度
    //   在低对比度场景下会放大增益 ⇒ 噪声也可能越过阈值 ⇒ 糊画面会变成
    //   一片白点，看着像坏了，反而干扰判断。要边缘叠加时把它改成 2'd2 重编即可。
    parameter [1:0] DISPLAY_MODE = 2'd0,

    // ---- ★ 帧缓存的存储分辨率（不是屏幕分辨率！）----------------
    // 屏幕永远是 640x480@60。这里说的是**帧缓存里真正存多少像素**，
    // 显示时用"整数最近邻放大"铺回 640x480（STORE_W=320 就是 2 倍放大）。
    //
    // ★★ 为什么必须能降采样：BRAM 只有 256 块 x 8192 bit = 2 097 152 bit，
    //    双缓存要装 2 帧，于是"每像素多少 bit"被分辨率锁死：
    //      640x480 双缓存 => 2097152 / 614400 = 3.41 bit/像素 => 只能 3bpp
    //                       （8 级灰度 + 4x4 抖动；抖动幅度 32/255 = 12.5%，
    //                         在实拍上就是肉眼可见的麻点，观感很差）
    //      320x240 双缓存 => 2097152 / 153600 = 13.6 bit/像素 => 8bpp 绰绰有余
    //                       （256 级灰度，不需要抖动、没有断层、没有麻点；
    //                         只用 150 块，还空出 106 块）
    //    代价只是"半分辨率"，而本工程的镜头本来就是失焦状态，
    //    半分辨率几乎看不出差别 —— 拿一半分辨率换回 8 bit 灰度和干净的画质，
    //    这笔账非常划算。
    parameter STORE_W = 640,
    parameter STORE_H = 480,

    // ★★ 第 9b 轮新增：**两块 bank 写同样的数据**（= 退回"单缓存"行为）。
    //   第 9 轮的实拍证明了一件事：画面里有真实场景结构 ⇒ 摄像头活着、
    //   降采样和 8bpp 都对。那"画面完全冻住"就只剩一种机制：
    //   **显示侧的 bank 交接链路断了**（done_toggle 的跨时钟边沿没被
    //   显示域看到 ⇒ rd_bank 永不切换 ⇒ 永远读同一块）。
    //
    //   WB_BOTH = 1 时，两块 bank 的写使能都直接用 fb_we_r，
    //   两块内容永远一样 ⇒ 显示侧读哪块都是最新画面。
    //   **冻结这条失效路径在结构上就不存在了**，代价是撕裂会回来
    //   （读指针扫过正在写的区域）—— 这正是旧版"有图像、有轮廓、
    //   但有撕裂"的行为，也就是用户明确要求退回去的那个状态。
    //
    //   改回 0 就恢复真正的乒乓双缓存（无撕裂，但依赖交接链路正常）。
    //
    //   ★ 第 9d 轮改回 0：冻屏的真正根因已查明并修掉 —— 不是乒乓本身，
    //     而是**帧边界信号（VSYNC）没在触发**，导致 bank 永不交接。
    //     帧边界已换成"行计数回绕"（见 6a 节），它只依赖"像素还在进来"，
    //     这一点已被画面本身证明。所以乒乓现在可以安全打开了。
    parameter WB_BOTH = 1'b0,

    // 每像素位深。8 = 全 8bit 灰度（推荐，配 STORE_W/H = 320/240）
    //              3 = 8 级灰度（只在坚持 640x480 双缓存时才需要，会有麻点）
    parameter FB_DW = 8,

    // 4x4 Bayer 有序抖动。**只在 FB_DW < 8 时才起作用**（FB_DW=8 时无效）。
    // 它把量化误差摊到 4x4 邻域，能把 8 级灰度"混"出接近 5~6 bit 的观感，
    // 代价是画面多一层 4x4 细密纹理。
    parameter DITHER = 1'b0,

    // ★★ 第 25n（重建）：彩色 RGB332 逐通道 Bayer 抖动（治漫画风/色带）。
    //   这是用户认可的"换模块后第一版"：满幅 4×4 Bayer，用**存储格坐标**
    //   （srow=row_d1[8:1]、scol=(pcol-2)>>1 的低 2 位）。色带消除效果好，
    //   副作用是 4×4 网格纹理（用户说"4×4 躁动"，先回到这里再另想办法）。
    //   满幅：R 丢 2bit→0..3，G/B 丢 3bit→0..7。1'b1=开（默认）。
    //   ★ 第 25ag 轮：重新打开（恢复 25n"画质最清楚、色彩最正确"状态）。
    parameter CDITH  = 1'b1,

    // ★ 底部 12 行"纯显示域棋盘格"自检带（二分手术刀）。
    //   它的诊断使命（证明显示+TMDS+HDMI 全活）第 9 轮已经完成，
    //   第 10 轮按用户要求关掉，还你完整的 480 行画面。
    parameter CHK_BAND = 1'b0,

    // ★★ 屏上调试 UI 总开关（第 10b 轮新增）：
    //     控制顶部"对焦指示条"（含左端 8px 恒白探针）+ 右上角"帧心跳方块"。
    //   1'b1 = 保留（调焦/查冻屏时用）；
    //   1'b0 = 全部收掉，640x480 纯灰度画面（比赛演示用这个）。
    //   两者都是纯显示域的 2 个 mux 分支，关掉不影响任何统计逻辑，
    //   对焦/心跳的内部统计照常在跑，随时可以翻回来。
    parameter DBG_UI  = 1'b0,

    // ★★ 锐化强度（第 11 轮新增）：非锐化掩模，T 型核（上+左右邻居）。
    //   sharp = h + SHARP*(3h - e - g - i)，h=写入像素，e/g/i=上/左/右邻居。
    //   2'd0 = 关闭；2'd1 = 经典温和锐化（推荐起点）；2'd2 = 强锐化（会有轻微镶边）。
    //   只作用于 mode 0/2 的灰度存储路径，Sobel(mode1) 和测试图案(mode3) 不受影响；
    //   对焦统计用的 mag_q 在锐化之前抽头，统计值不变。
    parameter SHARP   = 2'd0,

    // ★ 第 23 轮：自动对比度总开关。1=开（5b 节逐帧 min/max 拉伸），0=关（直通原始灰度）。
    //   关掉的原因：这层 FPGA 自动增益和 OV5640 的 AE 是**两套自动调节叠加**，
    //   物体伸进/拿出画面时，min/max 突变 → 增益一步跳变（最多 ×8）→ 与 AE 同步
    //   追亮度 → 产生剧烈的"呼吸闪"。关掉后只剩传感器 AE 一层，闪烁大幅减轻，
    //   且完全在 FPGA 内、不碰 sensor、零黑屏风险。
    parameter ACTR   = 1'b0,

    // ★ 第 25 轮：彩色模式。0=灰度（原行为），1=RGB332 彩色（8bit 装 R3:G3:B2）。
    //   RGB332 与灰度同是 8bit，帧缓存位宽/块数完全不变（150 块 RAM10），
    //   改动最小、零容量风险。颜色略粗（256 色），但足够比赛演示。
    //   彩色模式下锐化链路（读侧竖直/水平）天然被绕过 —— 见读侧解码。
    parameter COLOR  = 1'b1,

    // ★★ 第 25af 轮：分屏显示模式（题目第 4 项要求：左侧原始灰度 + 右侧边缘）。
    //   1'b0 = 关闭（默认，保持原有模式行为）。
    //   1'b1 = 分屏：左半屏存锐化灰度、右半屏存 Sobel 二值边缘。
    //   开启后自动走灰度路径（绕过 COLOR/RGB332），因为题目要求"原始灰度"而非彩色。
    //   帧缓存位宽/块数不变（两边都是 8bit），读侧无需改动。
    //   ★ 第 25ag：红边叠加（EDGE_COLOR=1）时本参数应=0（两模式互斥）。
    //   ★ 第 25ai 轮：基础五项演示 → SPLIT=1（左灰度 + 右 Sobel 边缘，纯灰度扣题）。
    //   ★ 第 25aq 轮：高阶挑战（彩色+红边）→ SPLIT=0（分屏关，走彩色路径）。
    parameter SPLIT  = 1'b0,
    parameter SPLITVIEW = 1'b1,   // 1=全分辨率左右分屏（左边缘图/右灰度）

    // ★★ 第 25ag 轮：3×3 中值滤波（高阶挑战 1）。1'b1=开，1'b0=关。
    //   插在"灰度化之后、Sobel 之前"，用 9 输入排序网络取中值，
    //   抑制椒盐噪声，让 Sobel 边缘更干净（题目明确"提升边缘检测质量"）。
    //   成本：+2 行缓存（2 块 RAM10）+ 约 300 LUT 排序网络，零 DDR、零乘法器。
    //   ★ 第 25ag 注：中值滤波两级实现（延迟+2拍）需单独谨慎做，先设 0，
    //     本轮先验证红边叠加（EDGE_COLOR），中值滤波下一轮单独上。
    parameter MEDIAN = 1'b0,

    // ★★ 第 25ag 轮：红色边缘叠加彩色输出（高阶挑战 5）。
    //   1'b1=开：全屏 RGB332 彩色 + Sobel 边缘像素染成红色（R=满,G=0,B=0）。
    //   写侧在算 RGB332 的同时判 isedge，边缘像素强制写红色 0xE0。
    //   帧缓存/读侧结构完全不变（8bit RGB332），零 BRAM 增量。
    //   与 SPLIT 分屏互斥（SPLIT 走灰度，本模式走彩色）；开本模式时 SPLIT 应=0。
    //   ★ 第 25ai 轮：基础五项演示 → 关闭红边（切回分屏）。做高阶挑战时再打开。
    //   ★ 第 25aq 轮：高阶挑战（彩色+红边）→ EDGE_COLOR=1。
    //     并把红边数据源从"未滤波的 isedge"换成"经时域滤波的 edge_final_r2"
    //     （见写侧 bl_disp），这样红边 = 现在调好的那套边缘（面部线条全、白点少）。
    parameter EDGE_COLOR = 1'b1,

    // ★ 第 11b 轮补充：边缘门控。实测全画面无差别锐化会把传感器噪点一起放大
    //   （暗部 AGC 增益高，颗粒被锐化后满屏斑驳，观感反而"更糊"）。
    //   现在只有 Sobel 梯度 mag_q 超过门限的像素才锐化，平坦区原样通过。
    //   mag_q 的尺度 ≈ 边缘灰度幅度：噪声约 ≤15，真实边缘 ≥30。
    //   24 = 噪声之上、真边缘之下的安全档。嫌"不够锐"调小，嫌"颗粒感"调大。
    parameter [9:0] SHARP_THR = 10'd24,

    // ★ 第 12 轮补充：亮像素不锐化。用户实测：键盘缝隙等强边缘处出现
    //   "白点闪烁" —— 原图已 220+ 的亮像素被正过冲顶到 255 饱和，
    //   AGC 逐帧微调时这圈饱和点忽大忽小 = 闪烁。
    //   亮处本来就没有细节可增强，h >= SHARP_HI 直接原样通过。
    //   200 的取值：白色按键 220~255 被跳过，中灰细节边缘（<200）照常锐化。
    parameter [7:0] SHARP_HI = 8'd200,

    // ★ 第 13 轮补充：过冲限幅。第 12 轮的 SHARP_HI 只挡住"本身已亮"的像素，
    //   挡不住"中灰像素贴着暗边"的情况：h=190、邻居 60 时 hp 高达 +120，
    //   190+120=255 照样饱和出白点。限幅后最多亮/暗 SHARP_CAP 个灰阶，
    //   强边缘（阶跃 150）仍被拉成 220 的硬边，但不再产生 255 的孤立白点。
    //   40 ≈ 强边缘增幅可见、白点消失的平衡点。嫌锐度弱调大，白点没消干净调小。
    parameter [7:0] SHARP_CAP = 8'd40,

    // ★★ 周期性给 OV5640 写 0x503D（内置测试图案）的开关。
    //   ★ 默认关闭。它在 2.8 s 一次的节奏上不停写传感器寄存器，
    //     是"传感器被写进死状态、VSYNC 不再翻转"的最大嫌疑（见 README 8i/8j 节）。
    //     它的诊断使命（区分"数字通路"和"光学"）第 8f 节已经完成，
    //     之后要复查再临时打开。
    parameter TPAT_EN = 1'b0
)(
    // ---- 板级 ----
    input  wire       clk24m,
    input  wire       rst_n,          // KEY0, 按下 = 0
    // ---- C P1：实时阈值按键（低有效，板上弱上拉）----
    input  wire       key_plus,       // K1 = ball K1 = GPIOL_P_07
    input  wire       key_minus,      // K2 = ball K2 = GPIOL_N_06

    // ---- 传感器 (DVP) ----
    input  wire [7:0] cmos_d,
    input  wire       cmos_href,
    input  wire       cmos_vsync,
    input  wire       cmos_pclk,
    input  wire       cmos_sdat_in,   // SCCB SDA 回读
    output wire       cmos_sclk,
    output wire       cmos_sdat_out,
    output wire       cmos_sdat_oe,
    output wire       cmos_xclk,
    output wire       cmos_ctl1,
    output wire       cmos_ctl2,
    output wire       cmos_ctl3,

    // ---- PLL ----
    input  wire       clk_sys,
    input  wire       clk_pixel,
    input  wire       clk_pixel_10x,
    input  wire       pll_locked,
    output wire       pll_rstn,

    // ---- HDMI (硬核 LVDS 串行器) ----
    output wire       hdmi_txc_oe,
    output wire [9:0] hdmi_txc_o,
    output wire       hdmi_txc_rst_o,
    output wire       hdmi_txd0_oe,
    output wire [9:0] hdmi_txd0_o,
    output wire       hdmi_txd0_rst_o,
    output wire       hdmi_txd1_oe,
    output wire [9:0] hdmi_txd1_o,
    output wire       hdmi_txd1_rst_o,
    output wire       hdmi_txd2_oe,
    output wire [9:0] hdmi_txd2_o,
    output wire       hdmi_txd2_rst_o,

    // ---- 状态灯（低电平点亮）----
    output wire [5:0] led,

    // ---- UART 诊断（板载 CH340N USB 转串口）----
    //   ★ E10 = GPIOT_P_16，bank 2B，1.8 V LVCMOS。端口名必须与
    //     camera_hdmi.peri.xml / .isf 里的名字逐字一致。
    //   ★ 依据：HDK 的 UART 例程（06-FPGA_HDL_Basic/03_FPGA_UART_Test_Bottom），
    //     其 P&R 产物 pinout.rpt 实证 fpga_txd.P 落在 E10 / GPIOT_P_16 / bank 2B
    //     （该例程 .isf 里那个 "H6" 是 2021 年旧工程残留，已被 peri.xml 覆盖）。
    //   ★ 115200-8-N-1，协议见本文件第 13 节。只发不收，故不加 rxd。
    output wire       fpga_txd,

    // ========================================================
    //  DDR3 接口（第 26 轮：把显示帧缓存从 BRAM 搬到 DDR3）
    // ========================================================
    //  ★ 下面这些端口名**必须与 camera_hdmi.peri.xml 里的 comp_gpio
    //    名字逐字一致**（addr[0] / dq[0] / clk_p …），否则 P&R 会报
    //    "pin ... not found in design"。这套名字是从官方
    //    example_top.v（Ti60_Demo 工程）原样抄来的，不要改。
    //
    //  ★ DDR 时钟来自 peri.xml 里的 ddr_pll（= PLL_BR0，参考 clk_sys 96MHz）
    //    四个输出：tdqss 384M / core 192M / tac 384M(动态相位) / twd 384M(相位4)
    //    它们是"顶层输入端口"，由接口设计器把 PLL 输出绑到这里。
    input  wire        tdqss_clk,
    input  wire        core_clk,
    input  wire        tac_clk,
    input  wire        twd_clk,
    input  wire        ddr_pll_lock,
    output wire        ddr_pll_rstn_o,
    output wire [2:0]  shift,
    output wire [4:0]  shift_sel,
    output wire        shift_ena,

    // ---- DDR 命令 / 地址 ----
    output wire [15:0] addr,
    output wire [2:0]  ba,
    output wire        we,
    output wire        reset,
    output wire        ras,
    output wire        cas,
    output wire        odt,
    output wire        cke,
    output wire        cs,

    // ---- DQ（x16，按官方拆成 i/o/oe 三态形式）----
    input  wire [15:0] i_dq_hi,
    input  wire [15:0] i_dq_lo,
    output wire [15:0] o_dq_hi,
    output wire [15:0] o_dq_lo,
    output wire [15:0] o_dq_oe,

    // ---- DM ----
    output wire [1:0]  o_dm_hi,
    output wire [1:0]  o_dm_lo,

    // ---- DQS / DQS_N ----
    input  wire [1:0]  i_dqs_hi,
    input  wire [1:0]  i_dqs_lo,
    input  wire [1:0]  i_dqs_n_hi,
    input  wire [1:0]  i_dqs_n_lo,
    output wire [1:0]  o_dqs_hi,
    output wire [1:0]  o_dqs_lo,
    output wire [1:0]  o_dqs_n_hi,
    output wire [1:0]  o_dqs_n_lo,
    output wire [1:0]  o_dqs_oe,
    output wire [1:0]  o_dqs_n_oe,

    // ---- DDR 差分时钟（常量 + DDIO 双沿生成，见 8b 节）----
    output wire        clk_p_hi,
    output wire        clk_p_lo,
    output wire        clk_n_hi,
    output wire        clk_n_lo,

    // ---- VREF 参考电压（只对应 peri.xml 的两个 1.5V SSTL 引脚，
    //      不参与任何逻辑，纯粹为了端口名与引脚名对齐）----
    input  wire        verf0,
    input  wire        vref1
);

    // ========================================================
    //  1. 传感器控制脚  ★ 关键，不要乱改
    // ========================================================
    // 模组 OPENSOC OV5640 V1.2 反插后的实际映射：
    //    板卡 CTL1 -> 模组 PWDN （高 = 掉电！）
    //    板卡 CTL2 -> 模组 NC
    //    板卡 CTL3 -> 模组 STB （高 = 待机）
    //    板卡 CTL0 -> 模组 RST （未连 FPGA）
    assign cmos_ctl1 = 1'b0;
    assign cmos_ctl2 = 1'b0;
    assign cmos_ctl3 = 1'b0;

    // XCLK 直接把 24 MHz 晶振喂给传感器
    assign cmos_xclk = clk24m;

    // ========================================================
    //  2. PLL 复位
    // ========================================================
    // 查 Efinity 文档 topics/concept_interface_pll.html 原文：
    //     "RSTN ... Active-low PLL reset signal"
    // 而 rst_n (KEY0) 也是低有效，所以直接接过去即可。
    // LVDS_TX 的 RST 则是高有效（见下面第 11 节）。
    assign pll_rstn = rst_n;

    // ========================================================
    //  3. SCCB / I2C 配置 OV5640
    // ========================================================
    wire        cfg_done;
    wire        ack_ok;
    wire        any_ack;
    wire [9:0]  rom_addr;
    wire [23:0] rom_data;

    ov5640_cfg_rom u_rom (
        .clk  (clk24m),
        .addr (rom_addr),
        .data (rom_data)
    );

    // --------------------------------------------------------
    //  3b. ★ 诊断：周期性打开 / 关闭 OV5640 内置彩色条测试图案
    // --------------------------------------------------------
    // 0x503D 是 OV5640 的 test pattern 控制寄存器（bit7 = 使能）。
    // 图案由**传感器内部数字电路**产生，光路完全不参与，所以它是一把
    // 「二分法手术刀」：
    //     彩色条清晰锐利  => 传感器 + DVP + FPGA 这条数字通路全好，
    //                        画面糊就只能是**镜头光学**的问题
    //     彩色条也糊      => 回来查 ISP 缩放 / DVP 采集时序
    // 每 2^26 个 24 MHz 周期（≈2.8 s）走一格相位，共 8 格（≈22.4 s 一轮），
    // **只有相位 0 才打开彩色条** —— 即"亮 2.8 s、歇 19.6 s"，占空比 12.5%。
    // 这样一次烧录就能看到两种状态，又不会每 5.6 s 就把摄像头画面抢走一半时间
    // （第一版做成 50% 占空比，盯着屏幕调镜头时被反复打断，很难受）。
    //
    // 握手时序见 i2c_ov5640.v 的端口注释：req 保持 -> done 拉起 -> req 撤销
    // -> done 自动清。不做握手会连续重写同一寄存器，把 SCCB 总线占满。
    reg [25:0] tpat_cnt;
    reg [2:0]  tpat_ph;      // 相位 0..7，只有 0 亮
    reg        tpat_pend;    // 请求待发（= ext_req）
    reg        tpat_seen;    // 已看到 ext_done（打一拍，避免与 ext_done 同拍竞争）

    wire       ext_done;
    wire       tpat_tick = (tpat_cnt == 26'd0);
    // ★ TPAT_EN=0 时彻底不发这条 I2C 请求（连相位机都不推进）
    wire       tpat_on   = TPAT_EN & (tpat_ph == 3'd0);

    always @(posedge clk24m or negedge rst_n) begin
        if (!rst_n) begin
            tpat_cnt  <= 26'h3FFFFFF;
            tpat_ph   <= 3'd7;      // ★ 从 7 起，开机 2.8 s 第一次 tick 就进入相位 0
                                    //   ⇒ 上电后很快能看到一次彩条（兼作"板子活着"）
                                    //   若复位成 0，则要等满一轮 22.4 s 才第一次出现
            tpat_pend <= 1'b0;
            tpat_seen <= 1'b0;
        end else begin
            tpat_cnt <= tpat_tick ? 26'h3FFFFFF : (tpat_cnt - 26'd1);
            if (tpat_tick && TPAT_EN) begin
                tpat_ph   <= (tpat_ph == 3'd7) ? 3'd0 : (tpat_ph + 3'd1);
                tpat_pend <= 1'b1;
            end
            tpat_seen <= ext_done;
            if (tpat_pend && tpat_seen) tpat_pend <= 1'b0;
        end
    end

    wire [7:0] tpat_val = tpat_on ? 8'h80 : 8'h00;

    i2c_ov5640 #(
        .CLK_DIV  (120),
        .REG_NUM  (374)
    ) u_i2c (
        .clk      (clk24m),
        .rst_n    (rst_n),
        .scl      (cmos_sclk),
        .sda_i    (cmos_sdat_in),
        .sda_o    (cmos_sdat_out),
        .sda_oe   (cmos_sdat_oe),
        .cfg_done (cfg_done),
        .ack_ok   (ack_ok),
        .any_ack  (any_ack),
        .rom_addr (rom_addr),
        .rom_data (rom_data),
        .ext_req  (tpat_pend),
        .ext_addr (16'h503D),
        .ext_data (tpat_val),
        .ext_done (ext_done)
    );

    // ========================================================
    //  4. DVP 采集（cmos_pclk 域）
    // ========================================================
    wire        cap_de;
    wire [15:0] cap_pixel;
    wire [9:0]  cap_x;
    wire [9:0]  cap_y;
    wire        cap_vs_toggle;
    wire [15:0] cap_line_len;

    dvp_capture #(
        .IMG_W     (IMG_W),
        .IMG_H     (IMG_H),
        .VSYNC_POL (1'b1)
    ) u_cap (
        .pclk      (cmos_pclk),
        .rst_n     (rst_n),
        .vsync     (cmos_vsync),
        .href      (cmos_href),
        .data      (cmos_d),
        .de        (cap_de),
        .pixel     (cap_pixel),
        .x_cnt     (cap_x),
        .y_cnt     (cap_y),
        .vs_toggle (cap_vs_toggle),
        .line_len  (cap_line_len)
    );

    // dvp_capture 的 x_cnt 比像素早一拍（de=1 时 x_cnt 已经 +1 了），
    // 自己重新数一个干净的列号，供行缓存/帧缓存寻址用。
    reg  [9:0] pcol;
    reg        hr_d;
    wire       href_hi = cmos_href & ~hr_d;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                     hr_d <= 1'b0;
        else                            hr_d <= cmos_href;
    end

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                          pcol <= 10'd0;
        else if (href_hi)                    pcol <= 10'd0;
        else if (cap_de && pcol == IMG_W-1)  pcol <= 10'd0;
        else if (cap_de)                     pcol <= pcol + 10'd1;
    end

    // ========================================================
    //  5. RGB565 -> 8bit 灰度
    // ========================================================
    // dvp_capture 拼出来的 pixel[15:0] = {R[4:0], G[5:0], B[4:0]}
    wire [4:0] r5 = cap_pixel[15:11];
    wire [5:0] g6 = cap_pixel[10:5];
    wire [4:0] b5 = cap_pixel[4:0];

    // 5/6/5 -> 8/8/8：低位复制高位，保证 0 和满量程都对得上
    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};

    // BT.601 亮度：Y = 0.299R + 0.587G + 0.114B
    // 整数近似 77/150/29，和正好 256，所以 >>8 后仍是 0..255
    wire [15:0] ysum  = (r8 * 8'd77) + (g8 * 8'd150) + (b8 * 8'd29);
    wire [7:0]  gray8 = ysum[15:8];

    // ========================================================
    //  6. 3 个行缓存 + 角色轮转，做 3x3 窗口
    // ========================================================
    // 一行 640x8bit = 5120 bit < 8192，所以每个行缓存只吃 1 块 RAM10。
    // 每行结束时把 slot 转一格：
    //      slot         = 正在写入的行缓存
    //      slot-1       = 上一行（读）
    //      slot-2       = 上上行（读）
    // 读写落在不同的 RAM 实例上，因此**不会出现同地址读写冲突**，
    // 这在简单双口 RAM 上是必须的。
    reg [1:0] slot;
    wire      line_end = cap_de && (pcol == IMG_W-1);

    // ========================================================
    //  6a. ★★★ 帧边界：用"行计数回绕"，**不再依赖传感器的 VSYNC 引脚**
    // ========================================================
    // 实测证据（第 9c 轮，用户实拍 + 口述）：
    //   * 画面几何完全正确 —— 键盘、手、鼠标都清清楚楚
    //     ⇒ row_d1 在老老实实地 0..479 往复，降采样/写地址全对；
    //   * 但顶栏对焦白段只有 8px（foc_raw 恒 0），而 foc_raw **只在 vs_rise 锁存**；
    //   * 心跳方块（也由 vs_rise 驱动）不闪。
    // 两件事放在一起，唯一的解释是：**VSYNC 没在触发**
    // （引脚没接通 / 极性不对 / 常电平 —— 反正不可信）。
    //
    // VSYNC 一死，连锁后果是：bank 永不交接（⇒ 冻屏，第 9 轮的现象）、
    // 对焦条恒 0、自动对比度永不更新、心跳方块不闪。
    //
    // 而"行计数回绕"只依赖一个事实：**像素还在进来** —— 这已经被画面本身证明了。
    // 所以它是永远可用的帧边界，而且**不新增任何假设**
    // （y_cnt 本来就是按 IMG_H=480 回绕的，这里只是把"回绕"显式拿来当帧边界）。
    reg [9:0] row_d1_d;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) row_d1_d <= 10'd0;
        else        row_d1_d <= row_d1;
    end
    // row_d1 = cap_y 打一拍；cap_y 每行结束 +1、到 479 回绕到 0。
    // row_d1 每行只变一次、其余 ~1000 拍保持不变，
    // 所以 (row_d1_d==479 && row_d1==0) **每帧恰好为真 1 拍**，判定窗口极宽。
    // ★ 479 = IMG_H-1（本设计 IMG_H 固定 480；若改 IMG_H 这里要跟着改）
    wire frame_end = (row_d1_d == 10'd479) && (row_d1 == 10'd0);

    // cap_vs_toggle（传感器 VSYNC 的翻转标志）已不再用于帧边界。
    // 保留连接是为了以后想诊断 VSYNC 时有现成的观测点。
    wire cap_vs_toggle_unused = cap_vs_toggle;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)           slot <= 2'd0;
        else if (frame_end)   slot <= 2'd0;                      // 每帧从 0 开始排
        else if (line_end)    slot <= (slot == 2'd2) ? 2'd0 : slot + 2'd1;
    end

    wire [1:0] r1sel = slot - 2'd1;      // 上一行
    wire [1:0] r2sel = slot - 2'd2;      // 上上行

    wire we0 = cap_de & (slot == 2'd0);
    wire we1 = cap_de & (slot == 2'd1);
    wire we2 = cap_de & (slot == 2'd2);

    // ========================================================
    //  5b. ★ 自动对比度（逐帧统计灰度 min/max，下一帧线性拉伸）
    // ========================================================
    // 传感器自带的自动曝光只保证"平均亮度"合适；遇到对比度很低的场景
    // （整个画面偏灰、明暗差只有几十级）时，直接显示几乎看不出东西，
    // Sobel 更是接近全黑。这里在 FPGA 里再补一层：
    //   统计一整帧灰度的 min/max，下一帧把 [slo, shi] 拉到满量程。
    // 增益只用 2 的幂（不需要除法器，几个 LUT 就能实现），
    // 而且当对比度实在太低（span < 16）时不动手，免得把噪声放大出来。
    reg [7:0] gmin, gmax;      // 本帧统计中
    reg [7:0] slo,  shi;       // 本帧生效的黑点/白点（来自上一帧统计）

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            gmin <= 8'hFF; gmax <= 8'h00;
            slo  <= 8'h00; shi  <= 8'hFF;
        end else if (frame_end) begin
            // 帧边界：把刚统计完的结果锁存给下一帧，同时复位统计器
            if (gmax > gmin + 8'd8) begin
                slo <= gmin;
                shi <= gmax;
            end
            gmin <= 8'hFF;
            gmax <= 8'h00;
        end else if (cap_de) begin
            if (gray8 < gmin) gmin <= gray8;
            if (gray8 > gmax) gmax <= gray8;
        end
    end

    wire [8:0] cspan = {1'b0, shi} - {1'b0, slo};

    // 增益选 2 的幂：span 越小放大越多；span < 16 认为画面本来就平，不动
    wire [1:0] csh = (cspan >= 9'd128) ? 2'd0 :
                     (cspan >= 9'd64)  ? 2'd1 :
                     (cspan >= 9'd32)  ? 2'd2 :
                     (cspan >= 9'd16)  ? 2'd3 : 2'd0;

    wire [9:0]  cred = ({2'b00, gray8} > {2'b00, slo})
                     ? ({2'b00, gray8} - {2'b00, slo}) : 10'd0;
    wire [12:0] cval = cred << csh;

    // 后面整条链路（行缓存 / Sobel / 帧缓存）都用拉伸后的灰度
    // ★ 第 23 轮：ACTR=0 时直通原始灰度 gray8，绕过自动对比度（消除呼吸闪）。
    wire [7:0] gray_act = (cval > 13'd255) ? 8'hFF : cval[7:0];
    wire [7:0] gray_c   = ACTR ? gray_act : gray8;

    wire [7:0] q0, q1, q2;

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_rb0 (
        .wr_clk(cmos_pclk), .wr_en(we0), .wr_addr(pcol), .wr_data(gray_c),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(q0));

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_rb1 (
        .wr_clk(cmos_pclk), .wr_en(we1), .wr_addr(pcol), .wr_data(gray_c),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(q1));

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_rb2 (
        .wr_clk(cmos_pclk), .wr_en(we2), .wr_addr(pcol), .wr_data(gray_c),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(q2));

    // ========================================================
    //  6b. ★ 第 25f 轮：彩色 2x2 均值（8bit 三通道行缓存，复用灰度结构）
    // ========================================================
    // ★★ 25d 事故复盘：上一轮用 **16bit** 行缓存存 RGB565，被综合器切成
    //   READ_WIDTH/WRITE_WIDTH=10 的位切片（map.v 可见 u_cb0/mem__D$2 只接
    //   cq0[9:0]）→ 读"上一行"错位 → 画面滚动。
    // 本轮对策：拆成 **R/G/B 三个 8bit 行缓存组**（每组 3 槽，与灰度
    //   u_rb0/1/2 的 DW=8 结构**完全同款**），综合结果必然一致，不会位切片。
    // 写数据用 r8/g8/b8（5/6/5 已扩展成 8bit，见第 5 节），平均后再转 RGB332。
    // 复用灰度的 slot 轮转与 we0/we1/we2、r1sel（"上一行"选择），零新增轮转逻辑。
    wire [7:0] crq0, crq1, crq2;
    wire [7:0] cgq0, cgq1, cgq2;
    wire [7:0] cbq0, cbq1, cbq2;

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cr0 (
        .wr_clk(cmos_pclk), .wr_en(we0), .wr_addr(pcol), .wr_data(r8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(crq0));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cr1 (
        .wr_clk(cmos_pclk), .wr_en(we1), .wr_addr(pcol), .wr_data(r8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(crq1));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cr2 (
        .wr_clk(cmos_pclk), .wr_en(we2), .wr_addr(pcol), .wr_data(r8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(crq2));

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cg0 (
        .wr_clk(cmos_pclk), .wr_en(we0), .wr_addr(pcol), .wr_data(g8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cgq0));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cg1 (
        .wr_clk(cmos_pclk), .wr_en(we1), .wr_addr(pcol), .wr_data(g8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cgq1));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cg2 (
        .wr_clk(cmos_pclk), .wr_en(we2), .wr_addr(pcol), .wr_data(g8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cgq2));

    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cb0 (
        .wr_clk(cmos_pclk), .wr_en(we0), .wr_addr(pcol), .wr_data(b8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cbq0));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cb1 (
        .wr_clk(cmos_pclk), .wr_en(we1), .wr_addr(pcol), .wr_data(b8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cbq1));
    line_buffer #(.WIDTH(IMG_W), .DW(8), .AW(10)) u_cb2 (
        .wr_clk(cmos_pclk), .wr_en(we2), .wr_addr(pcol), .wr_data(b8),
        .rd_clk(cmos_pclk), .rd_en(1'b1), .rd_addr(pcol), .rd_data(cbq2));

    // 读"上一行"当前列（与灰度 row1_now 同拍同列），用同一个 r1sel 选择
    wire [7:0] cr_prev = (r1sel == 2'd0) ? crq0 : (r1sel == 2'd1) ? crq1 : crq2;
    wire [7:0] cg_prev = (r1sel == 2'd0) ? cgq0 : (r1sel == 2'd1) ? cgq1 : cgq2;
    wire [7:0] cb_prev = (r1sel == 2'd0) ? cbq0 : (r1sel == 2'd1) ? cbq1 : cbq2;

    // ---- 两级流水：把"对齐"这件事讲清楚 ----------------
    // 时序（设第 n 拍 cap_de=1，列号 pcol=c_n）：
    //   n     : gray8 = G(c_n)，行缓存读地址 = c_n
    //   n+1   : gd1 = G(c_n)            <- gray8 打一拍
    //           row1_now = mem[c_n]     <- 行缓存同步读的固有 1 拍延迟
    //   ★ 这两个才对上：同一列 c_n。
    //   再往下，tap 链给出 c_n-1 / c_n-2，
    //   于是第 n+1 拍的窗口中心 = c_n-1 ...
    //   为了让窗口中心恰好是"写帧缓存的那一列"，把窗口中心再退一列，
    //   即中心列 = c_n - 2，对应的灰度是 gd2 = G(c_n-2)。
    //   这样第 n+1 拍就能一次性拿到：
    //        窗口 9 个值（中心列 c_n-2）+ 该列的灰度 gd2
    // 于是写地址列 = pcol - 2，写灰度 = gd2，完全对齐。
    // -----------------------------------------------------
    reg [7:0] gd1, gd2;
    reg       de_d1;
    reg [9:0] row_d1;
    // ★ 第 25 轮：彩色 RGB332 —— cap_pixel 也延迟 2 拍，与 gd2（pcol-2 列）对齐，
    //   这样写入帧缓存的 RGB 和灰度地址天然一致。
    reg [4:0] r5_d1, r5_d2;
    reg [5:0] g6_d1, g6_d2;
    reg [4:0] b5_d1, b5_d2;
    // ★ 第 25f 轮：上一行彩色的延迟链（与当前行 r5_d1/r5_d2 同节奏）。
    //   cr_prev 是"上一行、当前列"，再延迟 1/2 拍得到"上一行、写列+1 / 写列"，
    //   与当前行的 r5_d1/r5_d2（写列+1 / 写列）一起凑成 2x2 块。
    reg [7:0] crp_d1, crp_d2, cgp_d1, cgp_d2, cbp_d1, cbp_d2;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            gd1 <= 8'd0; gd2 <= 8'd0;
            r5_d1 <= 5'd0; r5_d2 <= 5'd0;
            g6_d1 <= 6'd0; g6_d2 <= 6'd0;
            b5_d1 <= 5'd0; b5_d2 <= 5'd0;
            crp_d1 <= 8'd0; crp_d2 <= 8'd0;
            cgp_d1 <= 8'd0; cgp_d2 <= 8'd0;
            cbp_d1 <= 8'd0; cbp_d2 <= 8'd0;
            de_d1 <= 1'b0;
            row_d1 <= 10'd0;
        end else begin
            gd1    <= gray_c;      // ★ 用拉伸后的灰度，Sobel 也跟着受益
            gd2    <= gd1;
            r5_d1  <= r5; r5_d2  <= r5_d1;   // RGB 与灰度同拍对齐
            g6_d1  <= g6; g6_d2  <= g6_d1;
            b5_d1  <= b5; b5_d2  <= b5_d1;
            crp_d1 <= cr_prev; crp_d2 <= crp_d1;   // 上一行，同节奏
            cgp_d1 <= cg_prev; cgp_d2 <= cgp_d1;
            cbp_d1 <= cb_prev; cbp_d2 <= cbp_d1;
            de_d1  <= cap_de;
            row_d1 <= cap_y;
        end
    end

    wire [7:0] row1_now = (r1sel == 2'd0) ? q0 : (r1sel == 2'd1) ? q1 : q2;
    wire [7:0] row2_now = (r2sel == 2'd0) ? q0 : (r2sel == 2'd1) ? q1 : q2;

    // ---- 3x3 tap 链（每行 2 个延迟寄存器）----
    // 移位输入用 gd1（已经和行缓存读数据对齐的那一路）
    reg [7:0] cur_t1, cur_t2;
    reg [7:0] r1_t1,  r1_t2;
    reg [7:0] r2_t1,  r2_t2;

    // 行首清一次，避免上一行末尾的像素串到本行开头。
    // ★ 原来写的是 (pcol == 0)，但 pcol==0 那一拍 de_d1 一定为 0
    //   （对齐拍上 pcol 的取值是 1,2,3,...），条件恒假 = 白写。
    wire tap_clr = de_d1 & (pcol <= 10'd1);
    wire tap_en  = de_d1 & ~tap_clr;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            cur_t1 <= 8'd0; cur_t2 <= 8'd0;
            r1_t1  <= 8'd0; r1_t2  <= 8'd0;
            r2_t1  <= 8'd0; r2_t2  <= 8'd0;
        end else if (tap_clr) begin
            cur_t1 <= 8'd0; cur_t2 <= 8'd0;
            r1_t1  <= 8'd0; r1_t2  <= 8'd0;
            r2_t1  <= 8'd0; r2_t2  <= 8'd0;
        end else if (tap_en) begin
            cur_t2 <= cur_t1;  cur_t1 <= gd1;
            r1_t2  <= r1_t1;   r1_t1  <= row1_now;
            r2_t2  <= r2_t1;   r2_t1  <= row2_now;
        end
    end

    // ---- 3x3 窗口 -----------------------------------------
    // 列方向：tap2 = 中心-1, tap1 = 中心, now = 中心+1
    //   a b c      a=(中心-1, 上上行)  b=(中心, 上上行)  c=(中心+1, 上上行)
    //   d e f      d=(中心-1, 上一行)  e=(中心, 上一行)  f=(中心+1, 上一行)
    //   g h i      g=(中心-1, 当前行)  h=(中心, 当前行)  i=(中心+1, 当前行)
    //   中心列 = pcol - 2
    wire [9:0] w_a = {2'b00, r2_t2};
    wire [9:0] w_b = {2'b00, r2_t1};
    wire [9:0] w_c = {2'b00, row2_now};
    wire [9:0] w_d = {2'b00, r1_t2};
    wire [9:0] w_e = {2'b00, r1_t1};
    wire [9:0] w_f = {2'b00, row1_now};
    wire [9:0] w_g = {2'b00, cur_t2};
    wire [9:0] w_h = {2'b00, cur_t1};
    wire [9:0] w_i = {2'b00, gd1};

    // Gx = (c + 2f + i) - (a + 2d + g)
    // Gy = (g + 2h + i) - (a + 2b + c)
    wire [10:0] gx_p = {1'b0, w_c} + {w_f, 1'b0} + {1'b0, w_i};
    wire [10:0] gx_n = {1'b0, w_a} + {w_d, 1'b0} + {1'b0, w_g};
    wire [10:0] gy_p = {1'b0, w_g} + {w_h, 1'b0} + {1'b0, w_i};
    wire [10:0] gy_n = {1'b0, w_a} + {w_b, 1'b0} + {1'b0, w_c};

    wire [11:0] gx = {1'b0, gx_p} - {1'b0, gx_n};
    wire [11:0] gy = {1'b0, gy_p} - {1'b0, gy_n};

    wire [11:0] gx_abs = gx[11] ? (~gx + 12'd1) : gx;
    wire [11:0] gy_abs = gy[11] ? (~gy + 12'd1) : gy;

    wire [12:0] mag    = {1'b0, gx_abs} + {1'b0, gy_abs};   // 最大 2040
    wire [9:0]  mag_q  = mag[11:2];                          // /4（对焦统计/锐化门控用，尺度不变）
    // ★ 第 25am 轮：sobel8 去掉 /4 压缩，改"不除 + 饱和"（min(mag,255)）。
    //   旧版 mag/4 把弱边缘（人脸鼻梁/嘴唇，mag 仅 20~60）压到 5~15，
    //   被阈值 48 全滤掉 → 面部细节检测不出来。不除后弱边缘直接 20~60，
    //   强边缘饱和 255，动态范围完整、弱纹理可见。
    wire [7:0]  sobel8 = (mag > 13'd255) ? 8'hFF : mag[7:0];

    // 3x3 窗口在帧的前 2 行 / 前 2 列是无效的（行缓存里还是上一帧的残留）
    //
    // ★★ 这里只能判 de_d1 一个，绝对不能再 & de_d2 & de_d3 ★★
    //    原因：RGB565 每个像素要在 D[7:0] 上分两个字节送，所以
    //    dvp_capture 的 cap_de 天生就是"一拍高、一拍低"（占空比 50%）。
    //    于是 de_d1 = de(t-1)、de_d2 = de(t)（同相）、de_d3 = de(t-1)，
    //    三个信号永远不可能同时为 1 —— 加上去就等于把写使能恒定为 0，
    //    帧缓存一个字节都不会被写，整屏只会显示 BRAM 上电的随机内容。
    //    （上板第一次看到的"满屏雪花噪点"就是这个原因。）
    //    de_d1 本身正好是"每像素恰好一拍"，且与行缓存的读数据同相，
    //    所以它单独就是唯一正确的写选通。
    // ★ 第 13 轮：上限放宽到 639 + 行窗口。关键推导：写帧缓存的地址是
    //   pcol-2、数据是延迟 2 拍的灰度，两者永远对齐 —— 所以摄像头第
    //   638/639 列的原始垃圾像素**从来不会被写进帧缓存**（落空），
    //   第 10 轮看到的斑点其实是"存储列 319 永远没人写、显示 BRAM 非零初值"。
    //   放宽到 639 后：pcol=638 把第 636 列的干净数据写进存储列 318，
    //   右缘真数据多一列，读侧只需处理最后 1 列（镜像），糊带缩到 2px。
    // ★ 第 13 轮：行窗口同理 —— 用户发现"最下面的细模糊线"是帧尾行
    //   （传感器收尾/消隐过渡行）的垃圾，对称地加上 row_d1<=477，
    //   存储行 239 不再写，读侧钳回 238。
    wire col_win = (pcol  >= 10'd2)   & (pcol  <= 10'd639);
    wire row_win = (row_d1 >= 10'd2)  & (row_d1 <= 10'd477);

    wire sobel_ok = de_d1
                  & row_win
                  & col_win;

    wire [7:0] sobel_v = sobel_ok ? sobel8 : 8'd0;

    // ========================================================
    //  ★ C P2：实时阈值（高阶挑战 #2）
    // ========================================================
    wire [7:0] thr_lo_run;
    wire [7:0] thr_hi_run;
    thr_ctrl #(
        .INIT_LO (8'd48),
        .INIT_HI (8'd64),
        .STEP    (8'd8),
        .GAP     (8'd16),
        .LO_MIN  (8'd8),
        .LO_MAX  (8'd232),
        .DEB_CNT (24'd1_048_576)
    ) u_thr (
        .clk       (cmos_pclk),
        .rst_n     (rst_n),
        .key_plus  (key_plus),
        .key_minus (key_minus),
        .thr_lo    (thr_lo_run),
        .thr_hi    (thr_hi_run)
    );

    // ========================================================
    //  6a-2. ★ 非锐化掩模（第 11 轮）：把镜头的"软"拉回来一点
    // ========================================================
    // ★ 先搞清楚"写入的像素"是窗口里的哪一个：
    //   由 gd1/gd2 与 tap 链的延迟代数可知，写入值 gd2 == w_h（当前行中心），
    //   它的真实邻居只有三个已经流过的：
    //       e = 上一行同列（w_e）、g = 左邻（w_g）、i = 右邻（w_i）。
    //   "下"邻居在下一行、还没进管线 —— 所以用 T 型核，不能用完整 4 邻域。
    //     sharp = h + SHARP*(3h - e - g - i)
    //   SHARP=1 时即 sharp = 4h - e - g - i（经典温和锐化）。
    //
    // ★ 边界处理：pcol>=3 保证左邻是真的（tap 链行首清零过）；
    //   窗口外（每帧最前 2 行/最前 2 列）直接回退原始 gd2，不锐化。
    // ★ 统计不变：对焦能量 mag_q 从原始灰度抽头，锐化只影响"存进帧缓存的值"。
    wire [12:0] h3  = {2'b00, w_h[7:0], 1'b0} + {4'b0000, w_h[7:0]};            // 3h ≤765
    wire [12:0] egi = {4'b0, w_e[7:0]} + {4'b0, w_g[7:0]} + {4'b0, w_i[7:0]};  // ≤765
    wire signed [13:0] hpT  = $signed({1'b0, h3}) - $signed({1'b0, egi});       // ±765
    wire signed [14:0] hpTg = (SHARP == 2'd0) ? 15'sd0 : hpT <<< SHARP;
    // ★ 第 13 轮：过冲限幅 —— 把锐化增量钳在 ±SHARP_CAP 内，消灭边缘白点/黑点。
    wire signed [14:0] cap_p = $signed({7'b0, SHARP_CAP});
    wire signed [14:0] hpTc = (hpTg >  cap_p) ?  cap_p :
                              (hpTg < -cap_p) ? -cap_p : hpTg;
    wire signed [14:0] sharp_s = $signed({7'b0, w_h[7:0]}) + hpTc;              // ±1785
    wire [7:0] sharp8 = (sharp_s < 0)        ? 8'h00 :
                        (sharp_s > 15'sd255) ? 8'hFF : sharp_s[7:0];
    wire       sharp_ok = de_d1 & (row_d1 >= 10'd2)
                        & (pcol >= 10'd3) & (pcol <= 10'd635);
    // ★ 第 11b 轮：边缘门控 —— 平坦区（mag_q 弱）不锐化，防止放大传感器噪点。
    // ★ 第 12 轮：亮像素门控 —— h >= SHARP_HI 不锐化，消灭强边缘的"白点闪烁"。
    wire       edge_gate = (mag_q > SHARP_THR) & (w_h[7:0] < SHARP_HI);
    wire [7:0] gd2_sharp = (sharp_ok & edge_gate) ? sharp8 : gd2;

    // ========================================================
    //  6b. ★ 对焦指示：整帧平均边缘能量
    // ========================================================
    // 原理：mag_q = (|Gx|+|Gy|)/4，对"平滑斜坡"而言它约等于 2×每像素灰度斜率，
    // 对"阶跃边"而言约等于台阶幅度。所以
    //     本帧 Σ mag_q / 像素数  =  整幅画面的"锐度"（acutance）
    // 镜头合焦时高频多、斜率大 -> 该值大；失焦时到处是平滑过渡 -> 该值小。
    // 做成屏幕顶部的条形，就能边转镜头边看，不用靠人眼判断糊不糊。
    //
    // 位宽：每帧有效像素 640x480 = 307200，mag_q 最大 255，
    //       最坏 255*307200 = 78.3e6 < 2^27，取 28 位累加器。
    // 归一化：>>18 = /262144 ≈ 除以像素数再 ×1.17，落在 0..298。
    //       再做上限截断到 40，正好对应 40*16 = 640 像素满格。
    reg [27:0] mag_acc;
    reg [9:0]  foc_raw;      // 0..40，本帧锐度
    reg        foc_tgl;      // 每帧翻转一次，供跨时钟域"安全取样"

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            mag_acc <= 28'd0;
            foc_raw <= 10'd0;
            foc_tgl <= 1'b0;
        end else if (frame_end) begin
            // 帧边界：先把统计结果交出去，再清零重新统计
            foc_raw <= (mag_acc[27:18] > 10'd40) ? 10'd40 : mag_acc[27:18];
            mag_acc <= 28'd0;
            foc_tgl <= ~foc_tgl;
        end else if (sobel_ok) begin
            mag_acc <= mag_acc + {18'b0, mag_q};
        end
    end

    // ========================================================
    //  6c. ★★ 摄像头"帧心跳"：整条通路里唯一能证明传感器还在出帧的信号
    // ========================================================
    // 用户第 8 轮的反馈是"连动都不动了，这一帧就冻在这儿"。
    // 画面冻结有两种完全不同的原因，看屏幕本身分不出来：
    //     a) 传感器停帧了       -> 帧缓存再也没被写 -> 显示侧一直读同一块
    //     b) 显示侧读地址卡住了 -> 帧缓存在更新，但读的永远是同一格
    // 加这个心跳块就能一刀切开：它在顶栏右下方（12x12），
    // **每 16 个摄像头帧翻转一次**（30 fps 下约 0.53 s 闪一次）。
    //     方块在闪  => 传感器还在出帧，问题在显示读侧；
    //     方块冻住  => 传感器停帧了（回到 I2C 初始化 / 供电 / 时钟一侧查）。
    // 它跨时钟只用 1 bit（frm_tgl），2FF 同步，风险为零。
    reg [3:0] frm_cnt;
    reg       frm_tgl;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            frm_cnt <= 4'd0;
            frm_tgl <= 1'b0;
        end else if (frame_end) begin
            frm_cnt <= frm_cnt + 4'd1;
            if (frm_cnt == 4'hF) frm_tgl <= ~frm_tgl;
        end
    end

    // ========================================================
    //  7. 显示模式选择
    // ========================================================
    // 正常运行不再轮换画面，持续保留真实摄像头画面及其边缘；
    // 如需复现独立诊断图，可在工程参数中临时改 DISPLAY_MODE。
    wire [1:0] mode_raw = DISPLAY_MODE;

    // ★ 传感器彩色条测试图案期间，强制停在 mode 0（纯灰度）。
    //   理由：彩色条是"数字通路 vs 光学"的判据，必须让用户一眼认出它。
    //   若不强制，彩色条会在 Sobel / 棋盘档里被"藏起来"，
    //   用户可能以为彩色条没生效。tpat_on 在 clk24m 域，这里打两拍同步过来；
    //   它 2.8 s 才变一次，完全不存在亚稳态风险。
    reg [1:0] tpat_sync;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) tpat_sync <= 2'b00;
        else        tpat_sync <= {tpat_sync[0], tpat_on};
    end
    wire       tpat_on_c = tpat_sync[1];
    wire [1:0] mode      = tpat_on_c ? 2'd0 : mode_raw;

    // ★ 第 25an 轮：双阈值磁滞（Canny 简化版）治"白点闪烁"。
    //   根因：sobel8 去 /4 后单一阈值 48 落在传感器时域噪声带（梯度 20~50 逐帧抖动），
    //   背景平坦区噪声随机越阈 → 白点逐帧闪烁。
    //   治法：强边缘 = sobel_v > SOBEL_HI；弱边缘 = SOBEL_THR < sobel_v <= SOBEL_HI，
    //         只有弱边缘的左邻或上邻已经是边缘（8 连通生长）才保留。
    //   孤立噪声点周围没有连续边缘 → 被 LO 档拒掉；真实边缘线连续 → 生长保留。
    //   实现：left_edge = 上一拍 edge_hyst；up_edge = 上一行同列 edge_hyst，
    //         用一行宽（640bit）移位寄存器做行延迟（640 个 FF，可忽略）。
    reg        edge_prev;                 // 左邻（上一拍）是否边缘
    reg [639:0] edge_row_sr;              // 上邻行延迟：深度 = 行像素数（640）
    wire       edge_hyst = (sobel_v > thr_hi_run)                    // 强边缘
                         | ((sobel_v > thr_lo_run) & (edge_prev | edge_row_sr[639]));
    wire       isedge = sobel_ok ? edge_hyst : 1'b0;
    // ---- 2026-11-02 v6：全分辨率 Canny（薄边）用于红色边缘 ----
    wire canny_de, canny_edge;
    canny_core #(.W(640), .GAUSS_EN(1)) u_canny (
        .clk     (cmos_pclk),
        .rst_n   (rst_n),
        .de      (de_d1),
        .din     (gd1),
        .thr_lo  (thr_lo_run),
        .thr_hi  (thr_hi_run),
        .de_out  (canny_de),
        .edge_o  (canny_edge)
    );
    // ========================================================
    //  ★ C P3：简易目标识别（高阶挑战 #6）
    // ========================================================
    wire [9:0] bbox_x0, bbox_x1, bbox_y0, bbox_y1;
    wire [1:0] shape_class;
    wire       shape_valid;
    shape_detect #(
        .IMG_W    (640),
        .ROI_X0   (10'd200), .ROI_X1 (10'd439),
        .ROI_Y0   (10'd150), .ROI_Y1 (10'd329),
        .MIN_DARK (18'd2000),
        .MIN_BOX  (10'd60),
        .DARK_THR (8'd80),
        .LINE_GAP (10'd3)
    ) u_shape (
        .clk         (cmos_pclk),
        .rst_n       (rst_n),
        .de          (de_d1),
        .gray        (gd1),
        .frame_start (frame_end),
        .bbox_x0     (bbox_x0), .bbox_x1 (bbox_x1),
        .bbox_y0     (bbox_y0), .bbox_y1 (bbox_y1),
        .shape_class (shape_class),
        .valid       (shape_valid)
    );
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            edge_prev   <= 1'b0;
            edge_row_sr <= 640'd0;
        end else if (de_d1) begin
            edge_prev   <= edge_hyst;
            edge_row_sr <= {edge_row_sr[638:0], edge_hyst};
        end
    end

    // ---- mode 3：内置测试图案（不经过摄像头，直接写进帧缓存）----
    // ★ 用「2x2 黑白棋盘」而不是灰阶阶梯。
    //   原因：灰阶阶梯的对比度不够，手机实拍 + 屏幕反光会把台阶压平，
    //   根本判断不了。纯黑/纯白是最大对比度，怎么拍都糊不掉，
    //   而且四个象限直接告诉我们**是哪一个方向坏了**：
    //        左上 = 黑      右上 = 白
    //        左下 = 白      右下 = 黑
    //   如果看到的是完整的四象限棋盘 → 帧缓存+显示时序+TMDS+HDMI 全部正确，
    //   问题必定在摄像头一侧。
    //   如果只看到左右两半（或上下两半）→ 对应方向的行/列寻址有问题。
    wire [4:0] test5 = ((pcol < IMG_W/2) ^ (row_d1 < IMG_H/2)) ? 5'd31 : 5'd0;

    // ★ 这里给的是 **8bit 原值**，不是最终存进缓存的位宽。
    //   因为抖动必须在"截位之前"做：先加一个 0..step-1 的 Bayer 偏移，
    //   再取高 FB_DW 位，才等价于把量化误差摊到 4x4 邻域上。
    //   用三元式而不是 case —— DISPLAY_MODE 是 parameter，综合时常量折叠，
    //   写 case 会报 "case condition never applies [VERI-1166]"。
    // ★ 第 11 轮：mode 0/2 的灰度改为 gd2_sharp（锐化后），其余档不变。
    // ★ 第 25 轮：COLOR=1 时 mode 0/2 直接存 RGB332（R3:G3:B2，取延迟 2 拍
    //   对齐 pcol-2 列的高位），Sobel/测试图档仍用灰度路径。
    // ★ 第 25f 轮：彩色 2x2 四点均值（替代 25c 的"取单点"）。
    //   四点 = 当前行{写列+1, 写列} + 上一行{写列+1, 写列}：
    //     R: r5_d1/r5_d2（5bit）+ crp_d1/crp_d2 的 [7:3]（= 上一行 r5 原始值，因 r8={r5,r5[4:2]}）
    //     G: g6_d1/g6_d2 + cgp_*[7:2]（= g6 原始值）
    //     B: b5_d1/b5_d2 + cbp_*[7:3]
    //   四点求和 +2 四舍五入再 >>2，把传感器像素噪声约降一半 → 颗粒变匀、闪点减少，
    //   且**不引入任何新的行缓存结构风险**（8bit 三通道，与灰度同款）。

    // ---- 4x4 Bayer 有序抖动表（上移，供彩色量化先用）----
    function [3:0] bayer4;
        input [1:0] br;
        input [1:0] bc;
        begin
            case ({br, bc})
                4'b0000: bayer4 = 4'd0;   4'b0001: bayer4 = 4'd8;
                4'b0010: bayer4 = 4'd2;   4'b0011: bayer4 = 4'd10;
                4'b0100: bayer4 = 4'd12;  4'b0101: bayer4 = 4'd4;
                4'b0110: bayer4 = 4'd14;  4'b0111: bayer4 = 4'd6;
                4'b1000: bayer4 = 4'd3;   4'b1001: bayer4 = 4'd11;
                4'b1010: bayer4 = 4'd1;   4'b1011: bayer4 = 4'd9;
                4'b1100: bayer4 = 4'd15;  4'b1101: bayer4 = 4'd7;
                4'b1110: bayer4 = 4'd13;  default: bayer4 = 4'd5;
            endcase
        end
    endfunction

    wire [6:0] r4sum = {2'b0, r5_d2} + {2'b0, r5_d1}
                     + {2'b0, crp_d2[7:3]} + {2'b0, crp_d1[7:3]} + 7'd2;   // 0..126
    wire [7:0] g4sum = {2'b0, g6_d2} + {2'b0, g6_d1}
                     + {2'b0, cgp_d2[7:2]} + {2'b0, cgp_d1[7:2]} + 8'd2;   // 0..254
    wire [6:0] b4sum = {2'b0, b5_d2} + {2'b0, b5_d1}
                     + {2'b0, cbp_d2[7:3]} + {2'b0, cbp_d1[7:3]} + 7'd2;
    wire [4:0] r5_avg = r4sum[6:2];      // >>2 → 5bit
    wire [5:0] g6_avg = g4sum[7:2];      // >>2 → 6bit
    wire [4:0] b5_avg = b4sum[6:2];

    // ---- ★ 第 25n（重建）：RGB332 逐通道满幅 Bayer 抖动（治漫画风）----
    //   Bayer 索引用**存储格坐标**：srow=row_d1[8:1] 的低 2 位 = row_d1[2:1]，
    //   scol=(pcol-2)>>1 的低 2 位 = pcol[2:1]-1（写拍 pcol 恒偶）。
    //   满幅：丢 2bit → bayer>>2(0..3)，丢 3bit → bayer>>1(0..7)。饱和防绕回。
    //
    // ---- ★ 第 25ag 轮：回到 25n 的纯存储格坐标（去掉 25ac 的 FRC 帧旋转）。
    //   25ac 的 FRC 用户反馈"点阵没变化 + 轻微蠕动"，无效还添乱 → 撤销。
    //   Bayer 索引用**存储格坐标**：srow=row_d1[8:1] 的低 2 位 = row_d1[2:1]，
    //   scol=(pcol-2)>>1 的低 2 位 = pcol[2:1]-1（写拍 pcol 恒偶）。
    //   这是用户认可的"画质最清楚、色彩最正确"的 25n 版本。
    // ★ 2026-10-20 B1：全分辨率下 Bayer 索引用像素坐标低 2 位
    wire [1:0] brow = row_d1[1:0];
    wire [1:0] bcol = pcol[1:0];
    wire [3:0] cbayer = bayer4(brow, bcol);
    wire [1:0] cbias2 = cbayer[3:2];     // 0..3（R 丢 2bit）
    wire [2:0] cbias3 = cbayer[3:1];     // 0..7（G/B 丢 3bit）
    // R: 0..31 + 0..3 → 6bit，量化取 [4:2]
    wire [5:0] r_q = {1'b0, r5_d2} + {4'b0, cbias2};
    wire [2:0] r3_d = r_q[5] ? 3'd7 : r_q[4:2];
    // G: 0..63 + 0..7 → 7bit，量化取 [5:3]
    wire [6:0] g_q = {1'b0, g6_d2} + {3'b0, cbias3};
    wire [2:0] g3_d = g_q[6] ? 3'd7 : g_q[5:3];
    // B: 0..31 + 0..7 → 6bit，量化取 [4:3]
    wire [5:0] b_q = {1'b0, b5_d2} + {2'b0, cbias3};
    wire [1:0] b2_d = b_q[5] ? 2'd3 : b_q[4:3];
    // 开关：CDITH=0 时退回纯截断（25i 行为）
    wire [2:0] r3q = CDITH ? r3_d : r5_d2[4:2];
    wire [2:0] g3q = CDITH ? g3_d : g6_d2[5:3];
    wire [1:0] b2q = CDITH ? b2_d : b5_d2[4:3];
    wire [7:0] rgb332 = {r3q, g3q, b2q};
    // ★★ 第 25aq 轮：红边叠加搬家 —— 这里不再叠加，改到写侧"时域滤波后"那拍做。
    //   原因：旧做法用未滤波的 isedge（只有空间磁滞）→ 红边里仍有闪烁白点、
    //   且线条稀疏（当时 HI 很高）。现在边缘管线已升级（sobel8 不除 +
    //   磁滞 HI=64 + 2bit 时域计数器），把这个"调好的"边缘接到红边才有意义。
    //   rgb_edge 保留为 rgb332 的别名（时序/对齐不变），真正的红边在
    //   写帧缓存的 bl_disp 那一拍（与 edge_final_r2 同拍，天然对齐）。
    wire [7:0] rgb_edge = rgb332;
    wire [7:0] px8 = (COLOR && (mode == 2'd0 || mode == 2'd2)) ? rgb_edge :
                     (mode == 2'd0) ? gd2_sharp :                        // 纯灰度（锐化）
                     (mode == 2'd1) ? sobel_v :                          // 纯 Sobel
                     (mode == 2'd2) ? (isedge ? 8'hFF : gd2_sharp) :     // 灰度 + 白边（锐化）
                                      {test5, test5[4:2]};               // 内置测试图

    // ========================================================
    //  7b. ★ 缓存 bank 交接（写侧，cmos_pclk 域）
    // ========================================================
    // 写口永远只写"当前这一帧那一块"，写满一整帧就把它交出去、自己换另一块。
    // 交接点严格落在**摄像头帧起始**（frame_end = 行计数回绕），所以交出去的必然是完整帧。
    //   ★ 第 9d 轮：帧边界从 VSYNC 派生的 vs_rise 换成 frame_end（见 6a 节）。
    //     这是"冻屏"的真正根因 —— VSYNC 不触发 ⇒ 永不交接 ⇒ 显示侧永远读同一块。
    //   注意 done_bank 与 done_toggle 必须**同拍变化**：
    //   读侧靠 done_toggle 的边沿去"知道什么时候可以采 done_bank"，
    //   两者同拍翻转，读侧采到的 done_bank 才一定属于这个边沿。
    reg wr_bank, done_bank, done_toggle;

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            wr_bank     <= 1'b0;
            done_bank   <= 1'b0;
            done_toggle <= 1'b0;
        end else if (frame_end) begin
            done_bank   <= wr_bank;        // 刚写满的那一块交给显示侧
            done_toggle <= ~done_toggle;
            wr_bank     <= ~wr_bank;       // 新的一帧写另一块
        end
    end

    // ========================================================
    //  7c. 4x4 Bayer 有序抖动表 —— ★ 已上移到 rgb332 之前（彩色量化先用）
    // ========================================================

    // ========================================================
    //  8. 写帧缓存（cmos_pclk 域）
    // ========================================================
    // 写的是 Sobel 3x3 窗口的**中心**像素：行 row_d1、列 pcol-2，
    // 选通 sobel_ok（= de_d1 & row_d1>=2 & pcol>=2）。
    //
    // 再打一拍把地址加法器和 Sobel 组合逻辑切开，96 MHz 下更从容。
    // ★ bank 位必须和数据/地址/使能**一起**延迟一拍，
    //   否则交接那一拍会写进刚换过去的新块，污染它。
    wire [3:0] bayer_v = bayer4(row_d1[1:0], pcol[1:0]);

    // 抖动量 0..(2^(8-FB_DW)-1)。
    // ★ FB_DW=8 时根本不需要抖动（本来就是全 8bit），显式给 0，
    //   免得被综合器当成"未知分支"留着。
    wire [4:0] dofs = (FB_DW >= 8) ? 5'd0                  :
                      (FB_DW == 5) ? {2'b00, bayer_v[3:1]}  :
                      (FB_DW == 4) ? {1'b0,  bayer_v}       :
                                     {bayer_v, 1'b0};

    // ★ 抖动只在 FB_DW<8 时才有意义；FB_DW=8 + DITHER=1 也要关掉，
    //   否则等于给满量程灰度再加偏移，白白多一层噪点。
    wire       dith_on = DITHER & (FB_DW < 8);
    wire [7:0] px_dith = dith_on ? (px8 + {3'b000, dofs}) : px8;

    // 存进缓存的最终值：取高 FB_DW 位（FB_DW=8 时就是 px_dith 原样）
    wire [FB_DW-1:0] px_st = px_dith[7 -: FB_DW];

    // ---- ★★ 2x2 降采样：640x480 -> 320x240 --------------------------
    // 这一段的全部目的就是把"每像素只有 3.41 bit"变成"每像素 8 bit"。
    //
    // 取点方式：只写**偶行 + 偶列**的中心像素。
    //   存储行 srow = row_d1 >> 1      （row_d1 = 2,4,...,476 -> 1..238）
    //   存储列 scol = (pcol - 2) >> 1  （pcol   = 2,4,...,638 -> 0..318）
    // 因为限制了"偶行偶列"，每个存储格**一帧恰好被写一次**，
    // 不会出现"同格被写两次、后写的覆盖前写的"那种不确定行为。
    //
    // ★ 边界（第 13 轮定稿）：存储行 0/239、存储列 319 永远不会被写
    //   （行窗口 2..477、列窗口 2..639 的算术结果），读侧用镜像/复制补齐，
    //   屏幕上看不出任何黑边或糊带。
    //   ★ 注意：pcol=638 写入存储列 318 的数据是延迟 2 拍后的第 636 列
    //     灰度（地址与数据天然对齐），摄像头行尾的原始垃圾像素落在
    //     pcol 639（奇数不写）/无处可写 —— 进不了帧缓存。
    //
    // ---- ★ 2026-10-20 B1：全分辨率 640×480，不再 2×2 降采样 ----
    //   写侧窗口：存储行 srow = row_d1（2..477）、列 scol = pcol-2（0..637）
    //   写字节数 = 476 × 638 = 303,688 / 帧（对比 320×240 的 75,922）
    wire [9:0] srow = row_d1;                   // 2..477
    wire [9:0] scol = pcol - 10'd2;             // 0..637

    // ---- ★★ 第 25ak 轮：上下分屏整幅对比（改自 25af 的左右劈开）----
    // 题目第 4 项："上半部分显示原始灰度、下半部分显示边缘检测结果"。
    // 25af 的左右劈开会把脸切成两半（用户反馈"只检测到一侧眼镜框"），
    // 上下分屏则上半屏=完整一整幅灰度、下半屏=完整一整幅边缘，人脸完整。
    // 写侧：灰度全幅写 u_fb（乒乓、无撕裂）；边缘全幅写 edge_fb（单缓存）。
    // 两者共享同一写地址 fb_addr，只是数据不同。
    // ★ 边缘是二值，只需 1bit（edge_hyst），省 BRAM。（edge_bin 已并入 edge_hyst）
    wire [7:0] gray_w   = SPLIT ? gd2_sharp : px_st;    // 灰度写数据（分屏=灰度，否则原样）

    // ★ 第 25ao 轮：时域一致性 AND 治"白点闪烁"。
    //   根因：传感器时域噪声的梯度区间（20~50）和面部弱边缘（48~96）重叠，
    //   纯空间阈值/磁滞无法彻底分离——噪声大的一帧亮成白点、小的一帧灭，逐帧闪。
    //   闪烁的定义就是"帧间跳变"，所以用上一帧同位置边缘做参考：
    //     edge_final = edge_hyst(本帧) & hist_edge(上帧)   —— 两帧都判边缘才显示。
    //   噪声点本帧有上帧无 → AND 掉；真实边缘两帧都在 → 保留。
    //   历史缓存存"本帧原始判定 edge_hyst"（不能存 AND 结果，否则自锁清零）。
    //   代价：运动边缘首帧被吃（1 帧延迟，30fps 下 33ms，可接受）。
    //   资源：1bit 帧缓存 9 块 RAM10（177+9=186 ≤ 256）。
    //   时序：hist_edge_q 读 fb_addr_r（第1级）后 1 拍在 fb_we_r2 拍返回；
    //        edge_hyst 经 edge_data_r(1拍)/edge_data_r2(2拍) 延迟对齐到同一拍。
    // ★ 第 25ap 轮：1bit AND → 2bit 饱和计数器（快进慢出）。
    //   25ao 上板后视频确认：单帧噪声没了，但残留"连续 2 帧都越阈"的半稳定
    //   噪声团（AE 增益维持，黑衣服/暗背景里的 3~8 像素小白团），AND 杀不掉。
    //   计数器规则：cnt_next = edge ? min(cnt+1,3) : max(cnt-1,0)
    //   显示条件 = (cnt_next >= 2)                    —— 连续 2 帧才准入（单帧噪声永不见）
    //            | (hist >= 2 & cnt_next >= 1)        —— 已显示的抗 1 帧跌落（真边缘不闪，
    //                                                    顺带治 AE 呼吸造成的边缘闪）
    //   效果：边缘消失时留 2 帧淡出（66ms，可接受）；资源 DW=2 → 约 20 块 RAM10。
    //  ⚠️ 第 28 轮清理：这里原先还留着一句旧 2bit 版的
    //       wire [1:0] hist_edge_q;
    //     第 25aq 把边缘历史从 2bit 计数器降回 1bit 后，真正的声明已经
    //     落到下面（"第 25aq" 那段）的 wire hist_edge_q; ，上面这句成了
    //     重复声明（iverilog 报 already declared in this scope）。
    //     Efinity 一直容忍它，但属于隐患，已删除。


    // 地址 = srow*640 + scol。640 = 512 + 128，用移位加法，不要乘法器。
    //   位宽核对：最大 477*640 + 637 = 305917 < 2^19 = 524288 ✓
    wire [18:0] fb_addr_c = ({srow, 9'b0}           // srow * 512
                           + {srow, 7'b0}           // srow * 128  -> *640
                           + {9'b0, scol});

    // 测试图案模式（mode 3）下不受 3x3 窗口的"前 2 行/前 2 列无效"限制，
    // 但地址同样要走 col_win/row_win（否则 scol/行 会下溢/越界，写到别的格子去）。
    wire fb_ok = (mode == 2'd3) ? (de_d1 & col_win & row_win) : sobel_ok;

    reg             fb_we_r;
    reg             fb_bank_r;
    reg [18:0]      fb_addr_r;
    reg [FB_DW-1:0] fb_data_r;
    reg             edge_data_r;      // ★ 边缘写数据（1bit，与 fb_data_r 同拍延迟）

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            fb_we_r    <= 1'b0;
            fb_bank_r  <= 1'b0;
            fb_addr_r  <= 19'd0;
            fb_data_r  <= {(FB_DW){1'b0}};
            edge_data_r<= 1'b0;
        end else begin
            fb_we_r    <= fb_ok;
            fb_bank_r  <= wr_bank;
            fb_addr_r  <= fb_addr_c;
            fb_data_r  <= gray_w;
            edge_data_r<= edge_hyst;    // ★ 25ao：存本帧原始边缘判定（延迟1拍，供时域AND）
        end
    end

    // ========================================================
    //  8c. ★ 第 25g 轮：时域降噪（帧间 IIR）—— 治"杂点一直动来动去"
    // ========================================================
    // 用户实测（25f）：2x2 空间平均后颗粒变匀了，但平坦区仍有彩色杂点
    // 在逐帧"跳舞"。那是**传感器的时域噪声**（每帧像素值的随机抖动，
    // 任何相机都有）被 RGB332 的粗量化放大 + 每帧原样刷新所致。
    // 治法（摄像头去噪的标准做法）：**帧间 50/50 混合**。
    //   写第 N 帧某像素时，从"历史缓存"读出上一帧同地址的值，
    //   新旧各取一半写回；历史缓存同步存混合结果（IIR 递推）。
    //   静止画面噪声每帧衰减一半（稳态 ≈ 0.58 倍），叠加 25f 的空间平均，
    //   总噪声约为基线的 0.3 倍 → "动来动去"大幅平静。
    //   代价：运动物体留 1~2 帧淡残影（30fps 下 33~66ms）。
    // ★ 结构安全：历史缓存是**独立的** 8bit 帧缓存，两个口都在 cmos_pclk
    //   域、只被写侧使用 —— 不碰显示读口、不碰行缓存对齐，无 25d 那类
    //   位切片/跨域风险（DW=8 与已验证的 frame_buffer 完全同款）。
    //   资源：+75 块 RAM10（150+9+75=234 ≤ 256 ✓）。
    reg [1:0] warmup;                 // 复位后前 2 帧直通（历史内容尚未有效）
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)                           warmup <= 2'd0;
        else if (frame_end && warmup != 2'd2) warmup <= warmup + 2'd1;
    end
    // 只在彩色 8bit（RGB332 布局 {R3,G3,B2}）下混合；其它配置直通
    // ★ 第 25ak：分屏模式存的是纯灰度 gd2_sharp（非 RGB332），时域混合会
    //   按 R3G3B2 位切分误混纯灰度 → 必须关。加 ~SPLIT。
    // ★ 2026-10-20 B1：全分辨率下时域 IIR 需要 307200×8bit = 300 块 RAM10，装不下
    //   （详见 _计划_B1全分辨率_20261018.md）。STORE_W>320 时关掉 IIR，直通。
    wire blend_raw = (STORE_W <= 320) & warmup[1] & COLOR & (FB_DW == 8) & ~SPLIT;

    // —— 二级流水：第 1 级发历史读地址，第 2 级数据到齐后混合 ——
    reg             fb_we_r2;
    reg             fb_bank_r2;
    reg [18:0]      fb_addr_r2;
    reg [FB_DW-1:0] fb_data_r2;
    reg             edge_data_r2;   // ★ 边缘写数据二级延迟（1bit，与 fb_data_r2 对齐）

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            fb_we_r2    <= 1'b0;
            fb_bank_r2  <= 1'b0;
            fb_addr_r2  <= 19'd0;
            fb_data_r2  <= {(FB_DW){1'b0}};
            edge_data_r2<= 1'b0;
        end else begin
            fb_we_r2    <= fb_we_r;
            fb_bank_r2  <= fb_bank_r;      // ★ bank 标签必须跟数据一起延后
            fb_addr_r2  <= fb_addr_r;
            fb_data_r2  <= fb_data_r;
            edge_data_r2<= edge_data_r;
        end
    end

    wire [FB_DW-1:0] hist_q;
    // RGB332 逐通道混合（+ 四舍五入）。
    // ★ 必须逐通道加，不能把 8bit 当整数加——会跨 R/G/B 位边界串色。
    // ★ 声明顺序：bl_data 必须在 u_hist 实例**之前**（本项目有过
    //   "先用后声明→隐式网络接地"的教训，见第 9 节注释）。
    wire [3:0] bl_rs = {1'b0, fb_data_r2[7:5]} + {1'b0, hist_q[7:5]} + 4'd1;
    wire [3:0] bl_gs = {1'b0, fb_data_r2[4:2]} + {1'b0, hist_q[4:2]} + 4'd1;
    wire [2:0] bl_bs = {1'b0, fb_data_r2[1:0]} + {1'b0, hist_q[1:0]} + 3'd1;
    // ★ 第 25i 轮：撤销 25h 的运动门控，改**固定 3:1**（新 75% / 旧 25%）。
    //   25h 实测两问题：① 轮廓像素的新旧差在阈值(3)附近来回跳 → "直通/混合"
    //   两种待遇反复切换 = 轮廓上"出现消失"的闪点；② 直通把**未降噪的原始值**
    //   写进历史 → 历史变脏 → 大块残色拖影清不干净。
    //   固定 3:1：所有像素同待遇（无模式切换 → 无闪点）；
    //   残影每帧衰减 75%（约 1 帧基本消失，比 50/50 的逐帧减半快一倍）；
    //   时域降噪保留（稳态噪声 ≈0.78 倍，叠加空间 0.5 ≈ 基线 0.39 倍）。
    wire [4:0] bl_r3 = {fb_data_r2[7:5], 1'b0} + {2'b0, fb_data_r2[7:5]}
                     + {2'b0, hist_q[7:5]}    + 5'd2;              // 3*新+旧+2, ≤30
    wire [4:0] bl_g3 = {fb_data_r2[4:2], 1'b0} + {2'b0, fb_data_r2[4:2]}
                     + {2'b0, hist_q[4:2]}    + 5'd2;
    wire [3:0] bl_b3 = {fb_data_r2[1:0], 1'b0} + {1'b0, fb_data_r2[1:0]}
                     + {1'b0, hist_q[1:0]}    + 4'd2;              // ≤14
    wire [FB_DW-1:0] bl_mix = {bl_r3[4:2], bl_g3[4:2], bl_b3[3:2]};  // >>2
    wire [FB_DW-1:0] bl_data = blend_raw ? bl_mix : fb_data_r2;   // 直通/混合

    generate
    if (STORE_W <= 320) begin : g_iir_320
        frame_buffer #(
            .DEPTH (76800),
            .DW    (FB_DW),
            .AW    (17)
        ) u_hist (
            .wr_clk (cmos_pclk),
            .we     (fb_we_r2),
            .waddr  (fb_addr_r2),
            .wdata  (bl_data),
            .rd_clk (cmos_pclk),
            .re     (fb_we_r),                   // 第 1 级发读地址
            .raddr  (fb_addr_r),
            .rdata  (hist_q)                     // 第 2 级与 fb_*_r2 同时有效
        );
    end else begin : g_iir_off
        // 全分辨率：IIR 需要 300 块 RAM10，装不下 → 直通（blend_raw=0）
        assign hist_q = {FB_DW{1'b0}};
    end
    endgenerate

    // ---- ★★ 第 25ao/25ap 轮：边缘时域历史缓存（2bit 计数器，快进慢出）----
    // 存"本帧更新后的边缘计数 cnt_next"，读出来是"上一帧同位置计数"。
    // 读地址用 fb_addr_r（第1级），hist_edge_q 在 fb_we_r2 拍返回，
    // 与 edge_data_r2（本帧原始判定，同拍）一起算 cnt_next。
    // ★ 复位后前 2 帧历史未有效（warmup），直通单帧判定，防止 BRAM 初值垃圾
    //   计数把整屏点亮成边缘。
    // 写口只在 SPLIT（分屏）下有效；非分屏不写也不影响。
    // 写口只在 SPLIT（分屏）或 EDGE_COLOR（红边）下有效；两者都不用时整条关掉。
    wire       edge_mode = SPLIT | EDGE_COLOR;                       // 需要时域边缘的模式
    wire       use_hist  = warmup[1];                                 // 复位后前 2 帧历史未有效
    // ★ 第 25aq：BRAM 吃紧（彩色模式要同时容纳 3 个 8bit 帧缓存：双显示 + 降噪历史，
    //   共 261 块 > 256 上限）。边缘历史从 2bit 计数器降回 1bit 两帧 AND（省约 10~29 块），
    //   在 25ao 实测"面部线条全 + 白点大减"，足够。前 2 帧用 use_hist 门控，
    //   防止 BRAM 初值垃圾把整屏点亮成红边。
    wire hist_edge_q;                                                 // 上一帧同位置是否边缘
    wire edge_final_r2 = use_hist & edge_data_r2 & hist_edge_q;       // 两帧都判边缘才显示

    frame_buffer #(
        .DEPTH (STORE_W*STORE_H),
        .DW    (1),
        .AW    (19)                          // ★ B1：全分辨率地址 19 位
    ) u_hist_edge (
        .wr_clk (cmos_pclk),
        .we     (fb_we_r2 & edge_mode),
        .waddr  (fb_addr_r2),
        .wdata  (edge_data_r2),              // 存本帧原始判定，供下帧参考
        .rd_clk (cmos_pclk),
        .re     (fb_we_r & edge_mode),
        .raddr  (fb_addr_r),
        .rdata  (hist_edge_q)
    );

    // ---- ★★ 第 25aq 轮：红边叠加（搬到写侧"时域滤波后"这一拍）----
    // 显示帧缓存写 bl_disp：边缘像素染纯红 0xE0（RGB332: R满 G=0 B=0），
    // 非边缘保留时域降噪后的彩色 bl_data。
    // ★ 时域历史缓存 u_hist 仍写干净的 bl_data（不染红），否则红边会被
    //   帧间 IIR 混合进历史 → 红边周围拖出红晕残影。
    wire [7:0] bl_disp = (EDGE_COLOR && canny_de && canny_edge) ? 8'hE0 : bl_data;

    // 注意：frame_buffer 的例化放在第 9 节之后 —— Verilog 里
    // "先用后声明" 会被综合器当成无驱动的隐式网络直接接地，
    // 帧缓存和整条显示通路都会被剪掉。务必保持顺序。

    // ========================================================
    //  9. 显示时序 640x480@60（clk_pixel 域）
    // ========================================================
    //  25.2 MHz / (800 x 525) = 60.0 Hz
    //  H: 640 有效 + 16 前肩 + 96 同步 + 48 后肩 = 800
    //  V: 480 有效 + 10 前肩 +  2 同步 + 33 后肩 = 525
    //  行/场同步都是负极性（VGA 标准）
    localparam H_TOTAL = 800;
    localparam H_SYNC  = 96;
    localparam H_BP    = 48;
    localparam H_ACT   = IMG_W;      // 640
    localparam V_TOTAL = 525;
    localparam V_SYNC  = 2;
    localparam V_BP    = 33;
    localparam V_ACT   = IMG_H;      // 480

    // 显式声明位宽，避免 "32 位表达式截断到 10 位" 的告警
    localparam [9:0] H_ACT0  = H_SYNC + H_BP;        // 144
    localparam [9:0] H_ACT1  = H_SYNC + H_BP + H_ACT; // 784
    localparam [9:0] V_ACT0  = V_SYNC + V_BP;        // 35
    localparam [9:0] V_ACT1  = V_SYNC + V_BP + V_ACT; // 515

    // clk_pixel 域复位：等 pll_locked 之后同步释放
    reg [3:0] drst_sync;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) drst_sync <= 4'b0000;
        else        drst_sync <= {drst_sync[2:0], pll_locked};
    end
    wire rst_n_d = drst_sync[3];

    reg [9:0] hcnt, vcnt;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 10'd0;
            vcnt <= 10'd0;
        end else if (!rst_n_d) begin
            hcnt <= 10'd0;
            vcnt <= 10'd0;
        end else if (hcnt == H_TOTAL-1) begin
            hcnt <= 10'd0;
            vcnt <= (vcnt == V_TOTAL-1) ? 10'd0 : vcnt + 10'd1;
        end else begin
            hcnt <= hcnt + 10'd1;
        end
    end

    wire hsync_l = ~(hcnt < H_SYNC);
    wire vsync_l = ~(vcnt < V_SYNC);
    wire hact    = (hcnt >= H_ACT0) && (hcnt < H_ACT1);
    wire vact    = (vcnt >= V_ACT0) && (vcnt < V_ACT1);
    wire de_disp = hact & vact;
    wire [9:0] dx = hcnt - H_ACT0;

    // ========================================================
    //  9a. ★ 读 bank：只在**显示帧起始**切换 —— 消撕裂的关键
    // ========================================================
    // 显示侧收到"新块写满了"的边沿后，先把它存进 pending，
    // 一定要等到自己的 **帧起始 (hcnt==0 && vcnt==0)** 才真正换 rd_bank。
    // 于是这一整帧显示的像素必然**全部来自同一块缓存**，
    // 不可能出现"上半屏新帧、下半屏旧帧"的撕裂线。
    //
    // ★ 为什么"把读指针挂到摄像头帧起始"那种做法治不了本
    //   上一版没有乒乓，只把行基址复位挂到摄像头帧起始上，
    //   想靠"读比写快"让读指针永远领先。它能成立的前提是
    //   **摄像头帧周期恒定**。而 OV5640 的自动曝光会动态改帧长（VTS），
    //   帧率一变，"读领先多少"就跟着变 ⇒ 撕裂线仍然满屏乱漂。
    //   所以单缓存无论怎么对齐都不可能稳，必须双缓存。
    //
    // 跨时钟域：done_bank 与 done_toggle 同拍变化。
    //   done_toggle 过两拍再检边沿，done_event 出现时
    //   done_bank 已经稳定 2 个 clk_pixel，不会采到新旧混合值。
    reg [1:0] done_t_s;
    reg       done_t_d, done_b_s1, done_b_s2;
    reg       pending, pending_bank, rd_bank;

    wire      done_event = done_t_s[1] ^ done_t_d;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            done_t_s     <= 2'b00;
            done_t_d     <= 1'b0;
            done_b_s1    <= 1'b0;
            done_b_s2    <= 1'b0;
            pending      <= 1'b0;
            pending_bank <= 1'b0;
            rd_bank      <= 1'b0;
        end else begin
            done_t_s  <= {done_t_s[0], done_toggle};
            done_t_d  <= done_t_s[1];
            done_b_s1 <= done_bank;
            done_b_s2 <= done_b_s1;
            if (done_event) begin
                pending      <= 1'b1;
                pending_bank <= done_b_s2;
            end
            if ((hcnt == 10'd0) && (vcnt == 10'd0)) begin
                // 帧起始：把"待用"的 bank 正式生效。
                // ★ 同拍收到 done_event 时直接用新块（last-assignment 生效），
                //   不会像"if(pending) 之后再置 pending"那样把这次交接吞掉。
                rd_bank      <= done_event ? done_b_s2 : pending_bank;
                pending      <= 1'b0;
                pending_bank <= done_event ? done_b_s2 : pending_bank;
            end
        end
    end

    // ---- ★★ 读地址：320x240 缓存 -> 640x480 屏幕的整数最近邻 2x 放大 ----
    // 每个存储格在屏幕上占 2x2 个像素。
    //   存储行 drow = dy_d >> 1     （dy_d = 0..479 -> 0..239）
    //   存储列 dcol = dx   >> 1     （dx   = 0..639 -> 0..319）
    // 地址 = drow*320 + dcol，同样用移位加法（320 = 256 + 64）。
    //   位宽核对：最大 239*320 + 319 = 76799 < 2^17 ✓
    //
    // ★ 为什么"1 拍读延迟"在这里不会造成画面偏移：
    //   相邻的偶/奇列 (2n, 2n+1) 映射到同一个存储格，所以
    //   mem[addr(dx-1)] == mem[addr(dx)] 恒成立，横向一点不差；
    //   纵向的 1 拍延迟由 de_disp_r 对齐掉了。上一版全分辨率时
    //   是"1:1 取点"，那才需要关心这个偏移。
    wire [9:0]  dy_d = vcnt - V_ACT0;
    // ---- ★★ 2026-10-20 B1：全分辨率 1:1 映射（640×480，无 2× 放大）----
    //   写侧窗口：存储行 2..477、列 0..637（scol = pcol-2）
    //   读侧：显示行 2..477 各取一行；列 dx 0..2 → dcol 0，dx 3..639 → dcol 1..637
    //   （边界列复制，保证 dcol 单调不减；每行 638 个取数，与写侧严格对齐）
    //   ⚠ SPLIT 分屏版仍按 320×240 构建（本 B1 源码只保证 SPLIT=0 / EDGE_COLOR=1）。
    wire [9:0]  drow = dy_d;                       // 0..479（仅 daddr/SPLIT 用）
    wire [9:0]  dcol = (dx < 10'd3) ? 10'd0 : (dx - 10'd2);   // 0..637
    wire [18:0] daddr = ({drow, 9'b0}              // drow * 512
                       + {drow, 7'b0}              // drow * 128 -> *640
                       + {9'b0, dcol});

    // ========================================================
    //  8b. ★★ 帧缓存搬到 DDR3（第 26 轮）
    //      原来是两块 320x240@8bpp 的 BRAM 乒乓：
    //          2 x 76800 x 8bit / 8192 = 150 块 RAM10
    //      现在整帧写进板载 DDR3，片内只留 1 行行缓冲（1 块 RAM10）。
    //      净效果：RAM10 约 252 -> 146，后面加功能才有地方放。
    //      必须放在这里 —— 上面 hcnt/vcnt/de_disp/dcol/dy_d 都声明好了。
    // ========================================================
    //  ★ 接口与 BRAM 版一一对应，只换实现：
    //      写：fb_we_r2 / fb_addr_r2 / bl_disp
    //      读：de_disp  / dcol（**不再用 daddr** —— DDR 是顺序流，不能随机寻址）
    //
    //  ★ 为什么读侧要改成"列地址 + 取数脉冲"：
    //    DDR 侧（axi4_ctrl）是**帧流式**接口：数据只能顺序吐出来，且不能重复读。
    //    而显示侧因为水平/垂直各 2x 放大，同一个存储格要用 4 次。解法：
    //      ① 每个存储列只从 DDR 取一次（fb_fetch 用 dcol != dcol_d 卡边沿），
    //         取到的数据同时写进行缓冲 lbuf 并直通输出（保持 1 拍读延迟）；
    //      ② 同一存储行的**第二个显示行**不再取数，直接复用 lbuf（垂直 2x）；
    //      ③ 同一列的第二个显示像素复用上一拍的数据（水平 2x，由显示侧 dcol 相同实现）。
    //    这样"一帧 76800 个数据"刚好被顺序消费一次，与 DDR 流长度严格对齐。
    //
    //  ★ 消撕裂：原设计靠 done_toggle/done_bank 跨时钟握手，读侧只在**显示帧
    //    起始**换 bank。DDR 版由 axi4_ctrl 内部的 4 块 buffer 指针自动给出同样的
    //    语义（写指针在 EOF 递增、读指针在 rframe_vsync 下降沿切到最新写满的那一块），
    //    而且缓冲更深（最多 3 帧），比原来更稳。
    //    ⚠️ rd_bank 仍然要保留 —— SPLIT 分屏模式的 u_edge0/u_edge1 还在用它。
    //
    //  ★ 只搬显示帧缓存。u_hist（时域降噪历史 76 块）和 u_hist_edge（边缘历史
    //    10 块）读写同域、又小，留在片内 BRAM；更重要的是**不往 DDR 时钟域
    //    塞逻辑** —— tac_clk 周期只有 2.604ns（余量 5.4%），是全设计最紧的域。
    //
    //  ★ 参数正确性硬判据：C_BURST_LEN * 16 必须整除一帧字节数
    //      64 * 16 = 1024 ； 76800 / 1024 = 75  ← 整除 ✓
    //    不整除的话最后一笔 burst 会越过帧尾，读侧多读无效字节 → 行缓冲错位。
    // ========================================================

    // ---- 8b.1 DDR 差分时钟（官方写法：常量电平 + 寄存器 + DDIO 双沿）----
    //   clk_p/n 的 hi/lo 恒定 0/1，由 tdqss_clk(384MHz) 双沿采样 → 384MHz 差分时钟。
    assign clk_p_hi = 1'b0;      // DDR3 时钟需要 180 度相移
    assign clk_p_lo = 1'b1;
    assign clk_n_hi = 1'b1;
    assign clk_n_lo = 1'b0;
    //   tac_clk 固定用 PLLOUT[2]（官方 example_top.v 原文就是 5'b00100）
    assign shift_sel = 5'b00100;
    //   ★ DDR PLL 的复位释放必须挂在**主 PLL 的 lock** 上（官方就是这么写的：
    //     assign ddr_pll_rstn_o = sys_pll_lock;）。ddr_pll 的参考时钟就是
    //     hdmi_pll 的 clk_sys，主 PLL 没锁之前给参考时钟没意义；
    //     反过来若挂 rst_n，ddr_pll 会在 clk_sys 还不稳时就启动。
    assign ddr_pll_rstn_o = pll_locked;

    // ---- 8b.2 clk_sys 域复位同步（AXI 互连 + DDR 控制器都跑在 clk_sys 96MHz）----
    //   要等"主 PLL 锁 + DDR PLL 锁"都成立才释放，否则 DDR 控制器会在
    //   tdqss/tac/twd 还没稳的时候就开始跑校准状态机。
    //
    //   ★★ 第 28 轮（2026-10-16 上板后新增）：POR + 校准重试
    //      —— 专治「首次上电校准不成功，必须按一次复位键才通过」。
    //      症状：上电后 led[4](cal_done) 常亮，但 led[5](cal_pass) 不亮；
    //            手按一次复位键立刻全亮。
    //      机理：两个 PLL 相继锁定后，这一行原本立刻把 DdrCtrl 的复位放开，
    //            而此时 DDR3 颗粒自身的上电初始化 / VREF 还没安定，软核
    //            校准状态机跑出来的结果不可信。手动按键 = "过几秒再复位
    //            一次"，所以能救回来。
    //      对策：① plls_lock 成立后，ddrc_rst_ok 再保持低 ~200 ms 才放行；
    //            ② 放行后 1 s 内 cal_pass 仍不来 → 自动再复位一次（≤3 次）。
    //            ⚠️ 只改"复位何时放开"，不碰任何接口 / 引脚 / 参数。
    //      （时序逻辑本体放在第 12 节 led 段末尾，因为它要用 cal_pass_lat。）
    wire plls_lock   = pll_locked & ddr_pll_lock;
    reg  ddrc_rst_ok;                    // 1 = 允许放开 DdrCtrl 复位（高有效逻辑）

    reg [3:0] ddr_rst_sync;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) ddr_rst_sync <= 4'b0000;
        else        ddr_rst_sync <= {ddr_rst_sync[2:0], plls_lock & ddrc_rst_ok};
    end
    wire rst_n_s = ddr_rst_sync[3];      // clk_sys 域低有效复位
    wire rst_s   = ~rst_n_s;             // 高有效，给 axi4_ctrl

    // ---- 8b.3 DDR 校准完成前不发任何命令 ----
    wire ddr_cal_done, ddr_cal_pass;
    reg  ddr_unlock;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) ddr_unlock <= 1'b0;
        else          ddr_unlock <= ddr_cal_done;
    end

    // ---- 8b.4 axi4_ctrl <-> DdrCtrl 之间的 AXI 互联信号 ----
    wire [3:0]   d_awid;    wire [31:0]  d_awaddr;  wire [7:0] d_awlen;
    wire         d_awvalid, d_awready;
    wire [127:0] d_wdata;   wire [15:0]  d_wstrb;   wire d_wlast;
    wire         d_wvalid,  d_wready;
    wire [3:0]   d_bid;     wire [1:0]   d_bresp;   wire d_bvalid, d_bready;
    wire [3:0]   d_arid;    wire [31:0]  d_araddr;  wire [7:0] d_arlen;
    wire         d_arvalid, d_arready;
    wire [3:0]   d_rid;     wire [127:0] d_rdata;   wire [1:0] d_rresp;
    wire         d_rlast,   d_rvalid,    d_rready;
    wire [3:0]   d_aid;     wire [31:0]  d_aaddr;   wire [7:0] d_alen;
    wire [1:0]   d_alock;   wire         d_avalid,  d_aready, d_atype;

    // ---- 8b.5 显示帧起始 + 从 DDR 取数脉冲 ----
    wire       disp_frame_start = (hcnt == 10'd0) && (vcnt == 10'd0);
    reg  [9:0] dcol_d;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) dcol_d <= 10'h3FF;
        else        dcol_d <= dcol;
    end
    //  取数条件 = 有效像素 × 非分屏下半屏 × （非分屏时只取"第一个显示行"）× 新列
    //    · 非分屏（SPLIT=0）：每个存储行有 2 个显示行，只在偶数显示行取数 → 垂直 2x 复用 lbuf
    //    · 分屏（SPLIT=1）：每个存储行只有 1 个显示行，垂直 1:1 → 每行都取
    //      且下半屏显示的是边缘缓存 edge_q（不是帧缓存），必须停止取数，否则 FIFO 被多消费
    //   ★ 2026-10-20 B1：全分辨率下写侧写了 476 行 × 638 列 = 303,688 B/帧，
    //     读侧也必须严格取 476×638 个字节（行 2..477、列 0..637），多取或少取都会错位。
    wire [9:0] frow_ok = dy_d;
    wire fb_fetch = de_disp
                  & ~(SPLIT & (dy_d >= 10'd240))
                  & (SPLIT ? 1'b1
                           : ((frow_ok >= 10'd2) & (frow_ok <= 10'd477)))
                  & (dcol != dcol_d);

    // ---- 8b.6 显示帧缓存（DDR 版；只支持 FB_DW = 8）----
    wire [FB_DW-1:0] fb_rdata;

    // ========================================================
    //  8b.6b ★★ 第 36 轮（2026-10-16 20:xx）：DDR 通路「注入自检」
    // ========================================================
    //  背景：串口缩略图给出了一份"横向高度相关、纵向完全无关"的画面，
    //  但把 DDR 通路**逐行读完**（ddr_frame_buffer.v / axi4_ctrl.v /
    //  W0_FIFO_8 / R0_FIFO_8 参数）之后，静态推理的结论却是"这条路必然
    //  是顺序、对称、自洽的"——读侧 araddr 每显示帧 flush 归零后按
    //  C_ADDR_INC 顺序读，写侧按同一 C_ADDR_INC 顺序写，两边每帧字节数
    //  严格都是 319x238 = 75,922。**推理与观测矛盾，就必须实测。**
    //
    //  这段自检把"写进 DDR 的字节"换成**我们已知的图案**，于是：
    //    · 回读的图案干净  ⇒ DDR 通路无罪，问题在摄像头/算法侧（源数据）；
    //    · 回读的图案错乱  ⇒ DDR 通路有罪，且残差图会直接指出错在哪。
    //
    // ================================================================
    //  ★ 第 38 轮**改写**（用户喊停复盘后的结论）：上一版的"周期 64 字节"
    //    图案是**坏选择** —— 缩略图是 5:1 盒式平均，一存储行 319 不是 5 的
    //    倍数，所以任何细周期图案在缩略图里都必然拍频，看不出真假。
    //    现在改成**纯斜坡**，并且不再靠缩略图判读，改成**片上逐字节自判**。
    //
    //  ① 图案：第 n 个字节 = (n + 常量) mod 256 —— 一条**线性斜坡**。
    //     · 斜坡是"盒式平均的不动点"（线性函数取均值还是线性），缩略图不会搅花；
    //     · 任何**丢字节 / 重复字节**都会让斜坡出现**台阶**（差分突变），一眼可判；
    //     · 整体常量偏移（例如读侧统一错 1 列）**不会**被误判 —— 判据是
    //       "差分是否恒定"，对整体平移免疫。这是**有意**的：那类错不影响出图。
    //
    //  ② 判据（在 rd_clk = clk_pixel 域，逐字节）：
    //       sc_diff = 回读字节 − 期望字节            （本拍）
    //       出错  <=>  sc_diff != sc_diff_d          （与上一拍相比变了）
    //     每个显示帧**头 64 字节只校准、不判定**（躲开帧边界 / RFIFO flush）：
    //       那段时间 RFIFO 可能空，rd_en 被忽略但 sc_cnt 仍在加，
    //       不排除掉就会假报错。
    //     sc_first = "至今没错过"的**粘性**标志；首次出错时把 sc_cnt>>14
    //     （16 KB 一档，0..4）锁进 sc_bkt，之后不再改。
    //
    //  ③ 报读：不靠串口、不靠显示器 —— 直接上灯（见 12 节 SELFTEST 分支）：
    //       led[5] 常亮   = 全程零错；
    //       led[5] 5.7Hz 闪 = 出过错；
    //       led[4:2]      = 首个出错字节所在的 16 KB 桶（0..4；零错时全灭）。
    //     桶号直接告诉我们"错在帧的哪一段"（一帧 75,922 B ≈ 74 笔 burst）：
    //       桶 0 = burst  0..15   （写侧一开头就错 / flush 错位）
    //       桶 1 = burst 16..31
    //       桶 2 = burst 32..47   （帧中段 ⇒ 时间相关：溢出/亚稳态嫌疑大）
    //       桶 3 = burst 48..63
    //       桶 4 = burst 64..73   （尾段 ⇒ flush 提前 / 末笔 burst 异常）
    //
    //  ④ 不再"每 64 帧翻转"，**每帧都注入**，判据稳定、一次烧录 3 秒出结论。
    //  ★ 正式出图前把 SELFTEST 改回 1'b0 即可（灯语义同时恢复原来的关卡条）。
    // ================================================================
    //  ★★ 第 39 轮（2026-10-16 22:0x）：由用户实测灯结果回推 ——
    //      led[2..4] 全灭(桶 000) + led[5] 闪烁 ⇒ **DDR 通路在帧内前 16 KB
    //      就出现了字节级异常**（若"没跑起来"led[5] 会常亮，故确系真错）。
    //      但此刻更要紧的是：**"快照闸门修好撕裂了没有"从未实测过**
    //      （20:29 那批 15 帧疑似烧的是无闸门版，且自检版出的是斜坡图、
    //        本来就不能用来判撕裂）。
    //      ⇒ 先关掉自检，回到"正常图像 + 快照闸门 + off-by-one 修复"，
    //        让用户烧一次、抓一帧 —— 相邻行 Pearson 是唯一的判据。
    //  诊断版随时可回：把 SELFTEST 改回 1'b1 重建即可（灯语义同步切换）。
    localparam SELFTEST = 1'b0;

    reg  [18:0] tb_addr_d;
    reg  [7:0]  tb_bcnt;      // 帧内字节序号（低 8 位就是斜坡值）

    wire tb_fwrap = fb_we_r2 & (fb_addr_r2 < tb_addr_d);   // 与 ddr_frame_buffer 同判据

    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            tb_addr_d <= 19'd0;
            tb_bcnt   <= 8'd0;
        end else begin
            // ★ 2026-10-17：必须只在**真正写字节**时更新，否则行消隐期间
            //   scol=(pcol-2)>>1 在 pcol=0 时等于 511，地址会先跳高再回落，
            //   每行都被误判成帧回绕（ddr_frame_buffer.v 里同一个坑，已修）。
            if (fb_we_r2) tb_addr_d <= fb_addr_r2;
            // 帧首（回绕那拍）归零；其余每写一个字节 +1。
            // 注意：回绕那一拍写出去的字节用的是**上一帧遗留的 tb_bcnt**，
            //       于是整帧变成 (n + 常量) mod 256 —— 只是整体平移一个常量，
            //       我们的"差分恒定"判据对此免疫，不影响结论。
            if (tb_fwrap)      tb_bcnt <= 8'd0;
            else if (fb_we_r2) tb_bcnt <= tb_bcnt + 8'd1;
        end
    end

    wire [FB_DW-1:0] tb_pat    = tb_bcnt;          // ★ 纯斜坡
    wire [FB_DW-1:0] ddr_wdata = SELFTEST ? tb_pat : bl_disp;

    // ----------------------------------------------------------------
    //  8b.6c 片上自检比对器（clk_pixel 域）
    //    ★ 声明必须放在 12 节 LED 之前 —— 否则会被当隐式线接地（本文件踩过）。
    // ----------------------------------------------------------------
    //  ⚠️ 关键对齐：`u_fb_ddr` 里 `rdata` 是**寄存器输出**
    //        if (rd_fetch) rdata <= rframe_data; else rdata <= lbuf[rd_col];
    //     ⇒ 在 `fb_fetch` 那一拍，`fb_rdata` 还是**上一拍的值**（lbuf 的内容），
    //        取回来的新字节要到**下一拍**才出现在 `fb_rdata` 上。
    //     所以必须把 `fb_fetch` 打一拍再用；否则比对的是 lbuf 残值，
    //     会**满屏假报错**。（"差分恒定"判据只能免疫**恒定**偏移，
    //     免疫不了这种"取错源"的错。）
    reg fb_fetch_d;
    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) fb_fetch_d <= 1'b0;
        else          fb_fetch_d <= fb_fetch;
    end

    reg  [16:0] sc_cnt;                            // 当前呈现在 fb_rdata 上的字节序号
    reg  [7:0]  sc_diff_d;                         // 上一拍的差分

    wire [7:0]  sc_rd   = fb_rdata;                // DDR 回读出来的字节
    wire [7:0]  sc_diff = sc_rd - sc_cnt[7:0];     // 回读 − 期望（sc_cnt 必须先声明！）

    reg         sc_armed;                          // 本帧过了 64 字节才开始判定
    reg         sc_first;                          // 粘性：至今没错过
    reg  [2:0]  sc_bkt;                            // 首个出错字节所在的桶 (cnt>>14)

    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) begin
            sc_cnt    <= 17'd0;
            sc_diff_d <= 8'd0;
            sc_armed  <= 1'b0;
            sc_first  <= 1'b1;
            sc_bkt    <= 3'd0;
        end else if (disp_frame_start) begin
            sc_cnt    <= 17'd0;
            sc_armed  <= 1'b0;                     // 新的一帧：重新校准
        end else if (fb_fetch_d) begin             // ★ 用打了一拍的取数沿
            sc_cnt    <= sc_cnt + 17'd1;           // 比完再推进序号
            sc_diff_d <= sc_diff;                  // 每拍都跟一下
            if (sc_cnt >= 17'd64) sc_armed <= 1'b1;
            if (sc_armed && sc_first && (sc_diff != sc_diff_d)) begin
                sc_first <= 1'b0;                  // 记住"错过了"，永不回退
                sc_bkt   <= sc_cnt[16:14];         // 首个错的位置（16 KB 一档）
            end
        end
    end

    wire sc_pass_lat = sc_first;                   // 常亮判据：全程零错

    ddr_frame_buffer #(
        .STORE_W     (STORE_W),
        .STORE_H     (STORE_H),
        .DW          (FB_DW),
        .AW          (19),                   // ★ B1：全分辨率地址 19 位
        .C_ID_LEN    (4),
        .C_BURST_LEN (64)                    // 64*16 = 1024；303688 字节/帧，10123 拍
    ) u_fb_ddr (
        .wr_clk     (cmos_pclk),
        .wr_rst_n   (rst_n),
        .we         (fb_we_r2),
        .waddr      (fb_addr_r2),
        .wdata      (ddr_wdata),             // ★ 第 36 轮：经自检注入多路选择
        .wr_frame_end (frame_end),           // ★ 2026-10-18：用摄像头帧边界产生写侧 EOF

        .rd_clk     (clk_pixel),
        .rd_rst_n   (rst_n_d),
        .rd_frame   (disp_frame_start),
        .rd_col     (dcol),
        .rd_fetch   (fb_fetch),
        .rdata      (fb_rdata),

        .axi_clk    (clk_sys),
        .axi_reset  (rst_s),

        .axi_awid    (d_awid),    .axi_awaddr (d_awaddr), .axi_awlen (d_awlen),
        .axi_awsize  (),          .axi_awburst(),         .axi_awlock(),
        .axi_awcache (),          .axi_awprot (),         .axi_awqos (),
        .axi_awvalid (d_awvalid), .axi_awready(d_awready),

        .axi_wdata   (d_wdata),   .axi_wstrb  (d_wstrb),  .axi_wlast (d_wlast),
        .axi_wvalid  (d_wvalid),  .axi_wready (d_wready),

        .axi_bid     (d_bid),     .axi_bresp  (d_bresp),
        .axi_bvalid  (d_bvalid),  .axi_bready (d_bready),

        .axi_arid    (d_arid),    .axi_araddr (d_araddr), .axi_arlen (d_arlen),
        .axi_arsize  (),          .axi_arburst(),         .axi_arlock(),
        .axi_arcache (),          .axi_arprot (),         .axi_arqos (),
        .axi_arvalid (d_arvalid), .axi_arready(d_arready),

        .axi_rid     (d_rid),     .axi_rdata  (d_rdata),  .axi_rresp (d_rresp),
        .axi_rlast   (d_rlast),   .axi_rvalid (d_rvalid), .axi_rready(d_rready)
    );

    // ---- 8b.7 AXI4 AW/AR 复用（DdrCtrl 只有一个命令口，读写轮流发）----
    AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) u_aw_ar_mux (
        .aclk_i     (clk_sys),
        .arst_i     (rst_s),

        .awid_i     (d_awid),
        .awaddr_i   (d_awaddr),
        .awlen_i    (d_awlen),
        .awvalid_i  (d_awvalid && ddr_unlock),    // 校准完成前不许发
        .awready_o  (d_awready),

        .arid_i     (d_arid),
        .araddr_i   (d_araddr),
        .arlen_i    (d_arlen),
        .arvalid_i  (d_arvalid && ddr_unlock),
        .arready_o  (d_arready),

        .aid_o      (d_aid),
        .aaddr_o    (d_aaddr),
        .alen_o     (d_alen),
        .atype_o    (d_atype),
        .avalid_o   (d_avalid),
        .aready_i   (d_aready)
    );

    // ---- 8b.8 DDR3 控制器 ----
    //   ⚠️ Native 通路（wr_*/rd_*）本工程不用，全部显式接常量，
    //      避免悬空输入被判成 X。axi_asize/aburst/alock 固定 128bit/INCR/0
    //      （官方 example_top.v 原文：asize=4, aburst=1, alock=0）。
    DdrCtrl u_ddr_ctrl (
        .clk           (clk_sys),
        .core_clk      (core_clk),
        .twd_clk       (twd_clk),
        .tdqss_clk     (tdqss_clk),
        .tac_clk       (tac_clk),
        .reset_n       (rst_n_s),

        .reset         (reset),
        .cs            (cs),
        .ras           (ras),
        .cas           (cas),
        .we            (we),
        .cke           (cke),
        .addr          (addr),
        .ba            (ba),
        .odt           (odt),

        .wr_busy       (),           .wr_addr      (32'b0),
        .wr_en         (1'b0),       .wr_addr_en   (1'b0),
        .wr_ack        (),           .wr_data      (128'b0),
        .wr_datamask   (16'b0),

        .rd_busy       (),           .rd_addr      (32'b0),
        .rd_addr_en    (1'b0),       .rd_en        (1'b0),
        .rd_valid      (),           .rd_ack       (),
        .rd_data       (),

        .shift         (shift),
        .shift_sel     (),
        .shift_ena     (shift_ena),

        .cal_ena       (1'b1),
        .cal_done      (ddr_cal_done),
        .cal_pass      (ddr_cal_pass),
        .cal_fail_log  (),
        .cal_shift_val (),

        .o_dm_hi       (o_dm_hi),     .o_dm_lo     (o_dm_lo),
        .i_dqs_hi      (i_dqs_hi),    .i_dqs_lo    (i_dqs_lo),
        .i_dqs_n_hi    (i_dqs_n_hi),  .i_dqs_n_lo  (i_dqs_n_lo),
        .o_dqs_hi      (o_dqs_hi),    .o_dqs_lo    (o_dqs_lo),
        .o_dqs_n_hi    (o_dqs_n_hi),  .o_dqs_n_lo  (o_dqs_n_lo),
        .o_dqs_oe      (o_dqs_oe),    .o_dqs_n_oe  (o_dqs_n_oe),
        .i_dq_hi       (i_dq_hi),     .i_dq_lo     (i_dq_lo),
        .o_dq_hi       (o_dq_hi),     .o_dq_lo     (o_dq_lo),
        .o_dq_oe       (o_dq_oe),

        .axi_aid       (d_aid),       .axi_aaddr   (d_aaddr),
        .axi_alen      (d_alen),      .axi_asize   (3'd4),
        .axi_aburst    (2'd1),        .axi_alock   (2'd0),
        .axi_avalid    (d_avalid),    .axi_aready  (d_aready),
        .axi_atype     (d_atype),

        .axi_wid       (8'b0),        .axi_wdata   (d_wdata),
        .axi_wstrb     (d_wstrb),     .axi_wlast   (d_wlast),
        .axi_wvalid    (d_wvalid),    .axi_wready  (d_wready),

        .axi_bvalid    (d_bvalid),    .axi_bready  (d_bready),
        .axi_bid       (d_bid),       .axi_bresp   (d_bresp),

        .axi_rvalid    (d_rvalid),    .axi_rready  (d_rready),
        .axi_rdata     (d_rdata),     .axi_rid     (d_rid),
        .axi_rresp     (d_rresp),     .axi_rlast   (d_rlast)
    );

    // ---- ★★ 第 25al 轮：边缘帧缓存（双缓存乒乓，1bit 二值）----
    // 上下分屏的下半屏整幅边缘。写侧 cmos_pclk 域、读侧 clk_pixel 域。
    // 第 25ak 是单缓存 → 读写竞争导致边缘闪烁；25al 改双缓存乒乓，
    // 复用灰度的 wr_bank/rd_bank（同帧同步交接），彻底消除边缘撕裂/闪烁）。
    // ★ 1bit 宽度只占 9 块/个（实测），两个 bank 仅 18 块，BRAM 完全够。
    // ★ 第 25aq：彩色+红边模式（SPLIT=0）下这套分屏边缘缓存用不到，但 PT
    //   不会自动剪（已实测 261>256 超容），必须用 generate 真正条件化掉，
    //   否则多占 20 块 RAM10 → 布局布线报 "Not enough physical locations"。
    wire edge_we0 = fb_we_r2 & SPLIT & ~fb_bank_r2;
    wire edge_we1 = fb_we_r2 & SPLIT &  fb_bank_r2;
    wire edge_q;                                  // 分屏下半屏边缘（非分屏恒 0）

    generate
    if (SPLIT) begin : g_split_edge_fb
        wire edge_q0, edge_q1;
        assign edge_q = rd_bank ? edge_q1 : edge_q0;

        frame_buffer #(
            .DEPTH (STORE_W*STORE_H),            // 76800
            .DW    (1),
            .AW    (17)
        ) u_edge0 (
            .wr_clk (cmos_pclk),
            .we     (edge_we0),
            .waddr  (fb_addr_r2),
            .wdata  (edge_final_r2),
            .rd_clk (clk_pixel),
            .re     (de_disp),
            .raddr  (daddr),
            .rdata  (edge_q0)
        );

        frame_buffer #(
            .DEPTH (STORE_W*STORE_H),            // 76800
            .DW    (1),
            .AW    (17)
        ) u_edge1 (
            .wr_clk (cmos_pclk),
            .we     (edge_we1),
            .waddr  (fb_addr_r2),
            .wdata  (edge_final_r2),
            .rd_clk (clk_pixel),
            .re     (de_disp),
            .raddr  (daddr),
            .rdata  (edge_q1)
        );
    end else begin : g_no_split_edge_fb
        assign edge_q = 1'b0;                     // 非分屏：不实例化边缘缓存
    end
    endgenerate

    // 帧缓存同步读有 1 拍延迟，所以 de / hsync / vsync 也要跟着打一拍
    reg de_disp_r, hsync_r, vsync_r;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            de_disp_r <= 1'b0;
            hsync_r   <= 1'b0;
            vsync_r   <= 1'b0;
        end else begin
            de_disp_r <= de_disp;
            hsync_r   <= hsync_l;
            vsync_r   <= vsync_l;
        end
    end

    // FB_DW bit -> 8bit：低位复制高位（把 2^FB_DW 级灰度线性铺回 0..255）
    //   FB_DW=8 -> 原样（256 级灰度，不需要抬）
    //   FB_DW=5 -> {v[4:0], v[4:2]}
    //   FB_DW=4 -> {v[3:0], v[3:0]}
    //   FB_DW=3 -> {v[2:0], v[2:0], v[2:1]}
    // ★ 第 25ak 轮：上下分屏时，下半屏（dy>=240）读边缘缓存 edge_q。
    //   边缘已存成 0xFF/0x00（8bit），直接显示；上半读灰度 fb_rdata。
    wire [7:0] fb_sel = (SPLIT && dy_d >= 10'd240) ? {8{edge_q}} : fb_rdata;   // 下半屏=边缘（1bit 扩 8bit）
    wire [7:0] gray_show = (FB_DW >= 8) ? fb_sel                               :
                           (FB_DW == 5) ? {fb_sel, fb_sel[4:2]}                :
                           (FB_DW == 4) ? {fb_sel[3:0], fb_sel[3:0]}           :
                                          {fb_sel[2:0], fb_sel[2:0], fb_sel[2:1]};

    // ★ 第 25 轮：RGB332 解码。缓存里 8bit = {R[2:0], G[2:0], B[1:0]}，
    //   每通道低位复制高位铺回 8bit：R={r,r,r,r,r[2:1],r[2:1]...} 近似。
    //   简单做法：3bit->8bit = {r, r, r, r, r, r[2], r[2], r[1]}；
    //             2bit->8bit = {b, b, b, b, b, b, b, b}。
    //   为省逻辑用"重复拼接"近似（观感足够）。
    wire [2:0] cr = fb_rdata[7:5];
    wire [2:0] cg = fb_rdata[4:2];
    wire [1:0] cb = fb_rdata[1:0];
    wire [7:0] r_show = {cr, cr, cr[2:1]};
    wire [7:0] g_show = {cg, cg, cg[2:1]};
    wire [7:0] b_show = {cb, cb, cb, cb};

    // ========================================================
    // 9b. ★ 顶部对焦指示条（clk_pixel 域叠加）
    // ========================================================
    // 跨时钟域：foc_raw 是 10 位多比特值，直接打两拍有可能在它"整帧更新"
    // 的那一拍采到新旧混合值。这里用"翻转标志 + 延迟取样"彻底规避：
    //   cmos 域在 frame_end 同拍更新 foc_raw 并翻转 foc_tgl；
    //   clk_pixel 域两拍同步 foc_tgl，再等一拍才真正锁存 foc_raw，
    //   此时数据已稳定 2 个 cmos_pclk + 2 个 clk_pixel（≈100 ns），
    //   远大于任何布线偏斜。取样只发生在帧边界，肉眼完全无感。
    reg [1:0] ftg_s;
    reg       ftg_d;
    reg [9:0] foc_lat;
    wire      foc_evt = ftg_s[1] ^ ftg_d;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            ftg_s   <= 2'b00;
            ftg_d   <= 1'b0;
            foc_lat <= 10'd0;
        end else begin
            ftg_s <= {ftg_s[0], foc_tgl};
            ftg_d <= ftg_s[1];
            if (foc_evt) foc_lat <= foc_raw;
        end
    end

    // mode 同步过来，仅用于"自检档不画条"
    reg [1:0] mode_s;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) mode_s <= 2'b00;
        else        mode_s <= mode;
    end

    // 摄像头帧心跳（1 bit）2FF 同步到显示域
    reg [1:0] frm_s;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) frm_s <= 2'b00;
        else        frm_s <= {frm_s[0], frm_tgl};
    end

    wire [9:0] bar_len = {foc_lat[5:0], 4'b0000};   // 0..1008，16 像素一档
    wire [9:0] bar_end = 10'd8 + bar_len;           // ★ 最少 8 像素：左端固定白块
    wire [9:0] dy_b    = vcnt - V_ACT0;

    // ========================================================
    //  ★ C P4：识别结果 cmos_pclk -> clk_pixel（翻转锁存）
    // ========================================================
    reg det_tgl;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) det_tgl <= 1'b0;
        else if (frame_end) det_tgl <= ~det_tgl;
    end
    reg [1:0]  dtg_s;
    reg        dtg_d;
    reg [9:0]  bb_x0_s, bb_x1_s, bb_y0_s, bb_y1_s;
    reg [1:0]  cls_s;
    reg        vld_s;
    wire       det_evt = dtg_s[1] ^ dtg_d;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            dtg_s <= 2'b00; dtg_d <= 1'b0;
            bb_x0_s <= 10'd0; bb_x1_s <= 10'd0;
            bb_y0_s <= 10'd0; bb_y1_s <= 10'd0;
            cls_s   <= 2'd0;  vld_s   <= 1'b0;
        end else begin
            dtg_s <= {dtg_s[0], det_tgl};
            dtg_d <= dtg_s[1];
            if (det_evt) begin
                bb_x0_s <= bbox_x0; bb_x1_s <= bbox_x1;
                bb_y0_s <= bbox_y0; bb_y1_s <= bbox_y1;
                cls_s   <= shape_class;
                vld_s   <= shape_valid;
            end
        end
    end
    // ★ 第 10b 轮：调试 UI 总开关。DBG_UI=0 时 bar_row / hb_row 全 0，
    //   对焦条、左端白探针、心跳方块一起从屏幕上消失，纯灰度画面。
    //   统计逻辑（mag_acc / foc_lat / frm_tgl）不受影响，照常在跑。
    wire       bar_en  = DBG_UI & (mode_s != 2'd3);  // 棋盘格自检档不叠加
    wire       bar_row = de_disp_r & bar_en & (dy_b < 10'd12);

    // ★★ 顶栏左端 8 像素**恒定白** —— 这是全工程最硬的一条探针：
    //   它不依赖帧缓存、不依赖摄像头、不依赖任何跨时钟域握手，
    //   只由 clk_pixel 的显示时序 + TMDS 编码 + LVDS 串行器构成。
    //   只要屏幕上能看到这一小段白，就说明显示通路从时序到 HDMI 全活着，
    //   黑屏就只可能出在"帧缓存 / 摄像头"一侧。
    //   之后接对焦条（白段长度 = 本帧锐度，16 像素一档），其余中灰底（量程刻度）。
    wire [7:0] bar_px = (dx < bar_end) ? 8'hFF : 8'h55;

    // ★★ 帧心跳方块：顶栏正下方、屏幕**右上角** 24x24 像素。
    //   第 9b 轮从 12x12 放大到 24x24 —— 太小的话隔着屏幕拍不出来。
    //   放在 dy_b 12..35 是为了不和上面那条 0..11 的对焦条打架
    //   （这样也不用去削短对焦条的量程）。
    //   亮 = 8'hFF / 暗 = 8'h08，对比度拉满，手机拍屏也分得清。
    //     闪  => 传感器还在出帧
    //     冻  => 传感器停帧（这一帧之所以不动就是因为它）
    wire       hb_row = de_disp_r & bar_en
                      & (dy_b >= 10'd12) & (dy_b < 10'd36) & (dx >= 10'd616);
    wire [7:0] hb_px  = frm_s[1] ? 8'hFF : 8'h08;

    // ★★ 最底部 12 行：**纯显示域 4x4 棋盘格**，只用 hcnt/vcnt 生成，
    //   完全不经过帧缓存、不经过摄像头、不经过 CDC —— 二分手术刀：
    //     看到棋盘格   => 显示 + TMDS + HDMI 全对，问题在帧缓存/摄像头一侧；
    //     看不到棋盘格 => 问题在显示时钟(PLL) / LVDS 串行器一侧。
    //   32x32 方块，足够大，手机拍屏也糊不掉。
    localparam [9:0] V_CHK0 = 10'd480 - 10'd12;     // 468（本设计固定 480 行）
    wire       chk_row = de_disp_r & CHK_BAND & (dy_b >= V_CHK0);
    wire       chk_v   = hcnt[5] ^ vcnt[5];
    wire [7:0] chk_px  = chk_v ? 8'hFF : 8'h10;

    // ★ 注意：这里**没有**任何"帧缓存还没填满就先出黑屏"的门控。
    //   上一版为了遮上电初期 BRAM 的随机内容加了 preview_valid 门控，
    //   结果把"摄像头没出帧"和"帧缓存没写进去"全都表现成一块纯黑屏，
    //   反而把问题藏起来了。BRAM 配置后初值就是 0，本来就是黑，不需要额外遮。
    // ========================================================
    //  ★ C P5：识别结果显示（白框 + 左上角状态块）
    // ========================================================
    wire in_x   = (dx >= bb_x0_s) && (dx <= bb_x1_s);
    wire in_y   = (dy_b >= bb_y0_s) && (dy_b <= bb_y1_s);
    wire on_xe  = (dx <= bb_x0_s + 10'd1) || (dx >= bb_x1_s - 10'd1);
    wire on_ye  = (dy_b <= bb_y0_s + 10'd1) || (dy_b >= bb_y1_s - 10'd1);
    wire frame_ = ((in_x & on_ye) | (in_y & on_xe));
    wire in_tag = (dx >= bb_x0_s) && (dx <= bb_x0_s + 10'd15) &&
                  (dy_b >= bb_y0_s) && (dy_b <= bb_y0_s + 10'd15);
    wire tag_ring = (dx <= bb_x0_s + 10'd2) || (dx >= bb_x0_s + 10'd13) ||
                    (dy_b <= bb_y0_s + 10'd2) || (dy_b >= bb_y0_s + 10'd13);
    wire tag_px   = in_tag & ((cls_s == 2'd2) ? 1'b1 : tag_ring);
    wire bbox_row = de_disp_r & vld_s & (frame_ | tag_px);
    wire [7:0] bbox_px = 8'h00;   // 灰度/分屏用黑色框，白纸上可见

    wire [7:0] gray_disp = bar_row ? bar_px :
                           hb_row  ? hb_px  :
                           chk_row ? chk_px :
                           bbox_row? bbox_px : gray_show;

    // ★ 第 25 轮：彩色模式下，R/G/B 三通道分别送解码后的分量；
    //   叠加条（bar/hb/chk，DBG_UI=0 时本来就不显示）仍走灰度。
    // ★ 第 25af：分屏模式（SPLIT=1）存的是灰度，必须强制走灰度显示，
    //   否则读侧会把这 8bit 灰度当 RGB332 解码成错误彩色。
    wire color_on = COLOR & ~SPLIT;
    wire ovl = bar_row | hb_row | chk_row | bbox_row;
    // ---- 2026-11-02：全分辨率左右分屏 ----
    //   左半屏 = 边缘图（Canny 红边像素 -> 白，其余黑）；右半屏 = 灰度（RGB332 亮度）
    //   1:1 取点，左/右各 320 列，没有 320->640 的 2x 放大，所以没有马赛克
    wire        split_left = (dx < 10'd320);
    wire        edge_px    = (fb_sel == 8'hE0);           // v6 Canny 红边
    wire [15:0] lum16       = r_show * 16'd77 + g_show * 16'd150 + b_show * 16'd29;
    wire [7:0]  lum8        = lum16[15:8];
    wire [7:0]  split_px    = split_left ? (edge_px ? 8'hFF : 8'h00) : lum8;
    wire [7:0] r_disp = SPLITVIEW ? split_px : (color_on ? (ovl ? gray_disp : r_show) : gray_disp);
    wire [7:0] g_disp = SPLITVIEW ? split_px : (color_on ? (ovl ? gray_disp : g_show) : gray_disp);
    wire [7:0] b_disp = SPLITVIEW ? split_px : (color_on ? (ovl ? gray_disp : b_show) : gray_disp);

    // ========================================================
    // 10. TMDS 编码
    // ========================================================
    // D0 = blue, D1 = green, D2 = red。
    // mode={2'b00, de}：de=1 走 video 编码，de=0 走 control 编码。
    //   D0 的 control 位 = {vsync, hsync}
    //   D1 / D2 的 control 位 = 00
    wire [2:0] tmode = {2'b00, de_disp_r};

    wire [9:0] tmds_d0, tmds_d1, tmds_d2;

    tmds_channel #(.CN(0)) u_enc_d0 (
        .clk_pixel        (clk_pixel),
        .video_data       (b_disp),
        .data_island_data (4'b0000),
        .control_data     ({vsync_r, hsync_r}),
        .mode             (tmode),
        .tmds             (tmds_d0)
    );

    tmds_channel #(.CN(1)) u_enc_d1 (
        .clk_pixel        (clk_pixel),
        .video_data       (g_disp),
        .data_island_data (4'b0000),
        .control_data     (2'b00),
        .mode             (tmode),
        .tmds             (tmds_d1)
    );

    tmds_channel #(.CN(2)) u_enc_d2 (
        .clk_pixel        (clk_pixel),
        .video_data       (r_disp),
        .data_island_data (4'b0000),
        .control_data     (2'b00),
        .mode             (tmode),
        .tmds             (tmds_d2)
    );

    // ========================================================
    // 11. HDMI / LVDS 输出
    // ========================================================
    // 时钟通道固定发 1111100000：在 252 Mbps 的位率下正好是
    // 25.2 MHz 方波（5 高 5 低），与 clk_pixel 同频。
    assign hdmi_txc_o   = 10'b1111100000;
    assign hdmi_txd0_o  = tmds_d0;
    assign hdmi_txd1_o  = tmds_d1;
    assign hdmi_txd2_o  = tmds_d2;

    assign hdmi_txc_oe  = 1'b1;
    assign hdmi_txd0_oe = 1'b1;
    assign hdmi_txd1_oe = 1'b1;
    assign hdmi_txd2_oe = 1'b1;

    // LVDS_TX 的 RST 是高有效（不同于 PLL 的 RSTN）。
    // 未锁定时保持复位，锁定后释放。
    assign hdmi_txc_rst_o  = ~rst_n_d;
    assign hdmi_txd0_rst_o = ~rst_n_d;
    assign hdmi_txd1_rst_o = ~rst_n_d;
    assign hdmi_txd2_rst_o = ~rst_n_d;

    // ========================================================
    // 12. 状态灯 = 「阶段关卡条」（低电平点亮）
    //
    //   ★ 上板时手边没有 HDMI 显示器，这 6 颗灯就是仪表盘：
    //     **每通过一关多亮一颗、常亮不灭**；按 KEY0(rst_n) 复位后重新逐关点亮。
    //     哪一颗不亮，卡点就在那一关，后面的一律不会亮。
    //
    //     led[0]  hdmi_pll 锁定 ................. pll_locked
    //     led[1]  ddr_pll 锁定 .................. ddr_pll_lock
    //     led[2]  SCCB 配置完成且每个字节都 ACK ... 摄像头被正确配置
    //     led[3]  DVP 曾经送进过像素 ............ 摄像头在出图
    //     led[4]  DDR 校准完成 (cal_done) ....... 校准跑完了
    //     led[5]  DDR 校准通过 (cal_pass) ....... ★ 阶段 4 最关键的一颗
    //             └ 常亮 = 通过；快闪 5.7 Hz = 跑完但没过；慢闪 1.4 Hz = 还没跑完
    //
    //   ★★ 第 28 轮（2026-10-16 上板后）新增两件事：
    //       12.1 上电「扫灯自检」：一次性把 led 序号 ↔ 物理位置对上（见下）。
    //       12.2 POR + 校准重试：治「首次上电校准不成功，须按一次复位键」。
    //
    //   ⚠️ LED6 / LED7（R3 / R4）在网络表里与 CMOS_D0 / CMOS_D1 **同一个球**，
    //      也就是说那两颗灯是被**摄像头的 D0/D1 数据线**驱动的，不是我们的逻辑，
    //      亮暗/闪烁都无意义 —— 现场看到"多出来两颗乱闪的灯"属正常。
    //
    //   ★ 灯序 = 数据流顺序：时钟 -> 配置 -> 采集 -> 校准 -> 显示。
    //   ★ led[0]/led[1] 是**纯组合**输出（不经过任何寄存器），所以哪怕
    //     PLL 没锁、所有时钟都死了，它们照样显示正确 —— 这是最底层的
    //     "板子活着吗 / 两个 PLL 分别锁没锁"。
    //     两颗分开是有意的：ddr_pll 是本轮新加的，是头号可疑对象。
    //   ★ led[2] 走 clk24m（板载 24M 晶振，只要有电就在跑）。
    //   ★ led[3] 走 clk_pixel，源信号在 cmos_pclk 域。
    //   ★ led[4]/led[5] 走 clk_sys（= DDR 所在域），这样哪怕显示链断了，
    //     校准结果照样看得见。
    //   ★ 相邻两颗可能在不同时钟域，但每颗各自独立，一颗坏了不影响别的。
    //
    //   ⚠️ 为什么不用官方那句注释掉的
    //        //assign led_o[1:0] = {w_ddr3_cal_pass, w_ddr3_cal_done};
    //      因为那样只能看校准，看不到"卡在 PLL 还是卡在摄像头"；
    //      而"卡在哪一关"才是没有显示器时最需要的信息。
    // ========================================================

    // ---- led[0] / led[1]：纯组合，不经寄存器 ----
    //   ★ 先落进 led_bar[5:0]，最后再经「上电扫灯」mux 送 led（见 12.1）。
    wire [5:0] led_bar;
    assign led_bar[0] = ~pll_locked;        // hdmi_pll：clk_sys / clk_pixel / clk_pixel_10x 的源
    assign led_bar[1] = ~ddr_pll_lock;      // ddr_pll ：tdqss / core / tac / twd 的源

    // ---- led[2]：clk24m 域（板载晶振，上电就在跑）----
    //   cfg_done / ack_ok 由 i2c_ov5640 输出，本来就是"一次成功就一直保持"
    //   的电平；i2c_ov5640 自己就跑在 clk24m 上，同域直连，不需要同步。
    // ★ 第 38 轮：SELFTEST=1 时 led[4:2] 改报"自检首错桶号"，不再报 SCCB/DVP/校准。
    //   （cal_pass 已单独闭合过；而"自检零错"本身就蕴含 DDR 校准通过。）
    assign led_bar[2] = SELFTEST ? ~sc_bkt[0] : ~(cfg_done & ack_ok);

    // ---- led[3]：DVP 有没有送进过像素（源在 cmos_pclk，落点在 clk_pixel）----
    //   先在 cmos_pclk 域把它压成"置 1 就永不清零"的电平（sticky），
    //   再两级同步进 clk_pixel。
    //   ⚠️ sticky **电平** 是安全的 CDC 形式；如果改成"每来一个像素产生一个
    //      单周期脉冲"再跨域，就会丢脉冲。所以这里必须是电平。
    reg px_seen_p;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n)      px_seen_p <= 1'b0;
        else if (cap_de) px_seen_p <= 1'b1;
    end

    reg [1:0] px_seen_s;
    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) px_seen_s <= 2'b00;
        else        px_seen_s <= {px_seen_s[0], px_seen_p};
    end
    assign led_bar[3] = SELFTEST ? ~sc_bkt[1] : ~px_seen_s[1];

    // ---- led[4] / led[5]：DDR 校准（clk_sys 域）----
    //   ddr_cal_done / ddr_cal_pass 由 DdrCtrl 在 core_clk 域产生，
    //   先两级同步进 clk_sys，再各自 sticky。
    //   ⚠️ 这是真实跨域路径，由 SDC 里的
    //      set_max_delay -from [get_clocks core_clk] -to [get_clocks clk_sys] 4.5
    //      约束覆盖（与已有的 ddr_unlock 是同一种路径）。
    reg [1:0] cal_done_sr, cal_pass_sr;
    reg       cal_done_lat, cal_pass_lat;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin
            cal_done_sr  <= 2'b00;
            cal_pass_sr  <= 2'b00;
            cal_done_lat <= 1'b0;
            cal_pass_lat <= 1'b0;
        end else begin
            cal_done_sr  <= {cal_done_sr[0], ddr_cal_done};
            cal_pass_sr  <= {cal_pass_sr[0], ddr_cal_pass};
            if (cal_done_sr[1]) cal_done_lat <= 1'b1;
            if (cal_pass_sr[1]) cal_pass_lat <= 1'b1;
        end
    end
    assign led_bar[4] = SELFTEST ? ~sc_bkt[2] : ~cal_done_lat;

    // led[5] 三态：常亮 = 校准通过 / 快闪 = 跑完但没过 / 慢闪 = 还没跑完
    //   ★ 第 27 轮（上板首测后）改：原来"还没跑完"这一态用的是**常灭**，
    //     结果它和"这颗灯/这个引脚是不是根本没在动"完全分不开 —— 上板时
    //     就会出现"某颗灯一直不亮，但我分不清是逻辑为 0 还是灯/引脚有问题"。
    //     现在三种状态**全都有动作**，绝不会"一直不亮"：
    //       常亮      -> 校准通过
    //       5.7Hz 快闪 -> 校准跑完但没通过
    //       1.4Hz 慢闪 -> 校准还没跑完（或没启动）
    //     因此：**只要哪颗灯真的一直不亮，它就不是 led[5]**（或该引脚有问题）。
    //   闪烁源：clk_sys 96 MHz 计数器分频
    //     bit23 -> 半周期 2^23 / 96e6 ≈ 87.4 ms  -> 5.7 Hz（快闪）
    //     bit25 -> 半周期 2^25 / 96e6 ≈ 349.3 ms -> 1.4 Hz（慢闪）
    reg [25:0] cal_blink;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) cal_blink <= 26'd0;
        else          cal_blink <= cal_blink + 26'd1;
    end

    // ★ 第 38 轮：SELFTEST=1 时 led[5] 改报"自检是否全程零错"，
    //   仍保持"永不常灭"的三态风格：
    //     常亮       -> 零错（DDR 写→DDR→读 整条通路逐字节干净）
    //     5.7Hz 快闪 -> 出过错（首错桶号见 led[4:2]）
    assign led_bar[5] = SELFTEST
                      ? (sc_pass_lat ? 1'b0 : ~cal_blink[23])
                      : (cal_pass_lat ? 1'b0             // 常亮      ：校准通过
                         : cal_done_lat ? ~cal_blink[23] // 5.7Hz 快闪：跑完了但没过
                                        : ~cal_blink[25]);// 1.4Hz 慢闪：还没跑完

    // ========================================================
    // 12.1 上电「扫灯」自检 —— 一次性把 led 序号 ↔ 物理位置对上
    // --------------------------------------------------------
    //  ★ 为什么需要：核心板那 8 颗灯**丝印上没有 LED0..LED7 编号**
    //    （用户手册只标了 "8bit LED" 一个大框），我们前两轮全靠
    //    "左起第几颗" 口头描述，已经对不上了。
    //
    //  ★ 时序（每次上电、每次按复位键之后各跑一次）：
    //        t ≈ 0.0 s ～ 0.5 s   先按**正常关卡条**显示（两颗 PLL 灯该亮就亮）
    //        t ≈ 0.5 s            关卡条收起来，6 颗**全灭**（一个明确的分界点）
    //        t ≈ 1.0 s            只有 led[0] 亮
    //        t ≈ 1.5 s            led[0..1] 亮
    //        t ≈ 2.0 s            led[0..2] 亮
    //        ……每 0.5 s 多一颗，直到 t ≈ 3.5 s 时 led[0..5] 全亮
    //        t ≈ 4.0 s            交回**正常关卡条**，此后不再扫
    //
    //  ★ 判读口诀：**从"全灭"之后开始数 —— 第 1 颗新亮的是 led[0]，
    //    第 2 颗是 led[1] …… 第 6 颗是 led[5]。** 记下它们的物理次序，
    //    以后就再也不用猜了。（下次想重看：按一下复位键即可重来。）
    //
    //  ★ 安全性：sweep_on 是寄存器，配置后初值 0。若 clk_sys 根本没跑，
    //    计数器不动、sweep_on 恒 0 → 直接显示 led_bar，两颗 PLL 灯仍是
    //    纯组合输出，"时钟全死也能看"这个特性没有被破坏。
    // ========================================================
    localparam SWEEP_HALF = 26'd48_000_000;    // 96 MHz × 0.5 s（2^26 = 67.1M > 48M ✓）
    localparam SWEEP_LAST = 5'd10;             // 共 10 步 = 5 s

    reg [25:0] sweep_tick;
    reg [4:0]  sweep_step;                     // 0 = 还没开始扫
    reg        sweep_on;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            sweep_tick <= 22'd0;
            sweep_step <= 5'd0;
            sweep_on   <= 1'b0;
        end else if (sweep_step == SWEEP_LAST) begin
            sweep_on   <= 1'b0;                // 扫完收工，永远交回关卡条
            sweep_tick <= 22'd0;
        end else if (sweep_tick == SWEEP_HALF) begin
            sweep_tick <= 22'd0;
            sweep_step <= sweep_step + 5'd1;
            sweep_on   <= 1'b1;
        end else begin
            sweep_tick <= sweep_tick + 22'd1;
        end
    end

    //   step 1 = 全灭（分界点）；step 2..7 = 依次 1..6 颗；step >=7 保持 6 颗
    wire [2:0] sw_n = (sweep_step >= 5'd7) ? 3'd6
                   : (sweep_step == 5'd0) ? 3'd0
                                          : (sweep_step[2:0] - 3'd1);
    wire [5:0] sweep_bar = (6'h01 << sw_n) - 6'h01;

    assign led = sweep_on ? ~sweep_bar : led_bar;

    // ========================================================
    // 12.2 POR + 校准重试（配 8b.2 里的 ddrc_rst_ok）
    // --------------------------------------------------------
    //  见 8b.2 的详细注释。这里只写"什么时候把复位放开 / 什么时候再来一次"：
    //    ① 两个 PLL 都锁 之后，先保持复位 POR_CYCLES(200 ms) 再放开；
    //    ② 放开后 WDOG_CYCLES(1 s) 内还没 cal_pass，就重新拉低复位再走一遍
    //       POR（最多 WDOG_LIMIT(3) 次自动重试），此后不再打扰。
    //  ⚠️ 这段只是"把用户手动按复位键那一下自动做掉"，不改变任何功能。
    // ========================================================
    localparam POR_CYCLES  = 25'd19_200_000;   // 96 MHz × 200 ms
    localparam WDOG_CYCLES = 27'd96_000_000;   // 96 MHz × 1 s
    localparam WDOG_LIMIT  = 2'd3;             // 最多自动重试 3 次

    reg [24:0] por_cnt;
    reg [26:0] wdog_cnt;
    reg [1:0]  wdog_retry;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            ddrc_rst_ok <= 1'b0;
            por_cnt     <= 25'd0;
            wdog_cnt    <= 27'd0;
            wdog_retry  <= 2'd0;
        end else if (!plls_lock) begin
            // 任一 PLL 没锁：保持复位，计时/计数全部清零
            ddrc_rst_ok <= 1'b0;
            por_cnt     <= 25'd0;
            wdog_cnt    <= 27'd0;
            wdog_retry  <= 2'd0;
        end else if (!ddrc_rst_ok) begin
            // ① POR 保持期：凑满 POR_CYCLES 再放行
            wdog_cnt <= 27'd0;
            if (por_cnt == POR_CYCLES) ddrc_rst_ok <= 1'b1;
            else                       por_cnt     <= por_cnt + 25'd1;
        end else if (!cal_pass_lat && (wdog_retry != WDOG_LIMIT)) begin
            // ② 已放行：1 s 内仍没 cal_pass → 重新复位一次
            if (wdog_cnt == WDOG_CYCLES) begin
                wdog_cnt    <= 27'd0;
                wdog_retry  <= wdog_retry + 2'd1;
                por_cnt     <= 25'd0;
                ddrc_rst_ok <= 1'b0;
            end else begin
                wdog_cnt <= wdog_cnt + 27'd1;
            end
        end
    end

    // ========================================================
    // 13. UART 软显示器（板载 CH340N，115200-8-N-1）
    // --------------------------------------------------------
    //  ★ 用途：手边没有 HDMI 显示器时，把 DDR 帧缓存里的画面缩成
    //    64x48 缩略图，用 ASCII hex 从串口打回电脑，由上位机 Python
    //    脚本重建成 PNG —— 等于一台"软显示器"。
    //
    //  ★ 线上协议（全部可打印 ASCII）：
    //       '@' <2位hex帧号> <2位hex状态字>
    //           <8位hex相机帧周期> <6位hex行周期> <6位hex显示帧周期> <2位hex保持数>
    //           CR LF                                     <- 帧头（27 字符 + CRLF）
    //       48 行：每行 64 个像素，每像素 2 位 hex（128 字符）+ CR LF
    //   一帧 = 29 + 48*130 = 6269 字符，115200 bps 下约 0.54 s。
    //   ★ 遥测字段（2026-10-18 接手后新增）：clk_sys 96MHz 计数值，见 13.0a。
    //     老上位机脚本只读 [1:5]，多余字段不影响它们；新脚本按 13/19/25/27 切。
    //   帧号每帧 +1，所以"串口有数据且在变"本身就是"系统在跑"的证据。
    //   状态字 bit0..7 = hdmi_pll / ddr_pll / cal_done / cal_pass /
    //                    cfg_done / DVP / any_ack / (恒1 版本标记)
    //   —— 也就是说：**校准过没过、摄像头配没配上，串口里直接写成字**，
    //      不用再靠数灯猜。
    //
    //  ★ 采样：显示输出 640x480，按 **10x10 盒式平均**降到 64x48
    //    （不是点抽样！点抽样 + 存储路径上的 4x4 抖动 = 一片雪花，判不了画面）。
    //    取的是 fb_rdata（DDR 帧缓存的读出口），也就是**最终画面**。
    //    写出值 = 0.78 x 方块平均灰度（线性），上位机乘 1.28 还原。
    //
    //  ★ 缩略图存在一块真正的双口 BRAM 里：
    //      写侧 clk_pixel（显示域）/ 读侧 clk_sys（UART 域）
    //    64*48 = 3072 B = 3 块 RAM10。两个时钟访问同一数组，综合器
    //    推断为 true dual port（原 frame_buffer.v 就是这么做的）。
    // ========================================================

    // ---- 13.0 状态字：给上位机的一行状态，不用再靠灯猜 ----
    //   这里要跨域取两个信号（都是"置位后永不清零"的静态电平，2 级同步即可）：
    //     cfg_done / ack_ok   <- clk24m   （i2c_ov5640 的输出）
    //     px_seen_p           <- cmos_pclk（第 12 节里 DVP 有没有来像素）
    //   ⚠️ 这两条跨域路径已被 SDC 的
    //        set_clock_groups -asynchronous -group {clk24m}
    //            -group {cmos_pclk} -group {clk_sys clk_pixel clk_pixel_10x}
    //      覆盖（clk24m / cmos_pclk / clk_sys 分属不同组 = 互不分析）。
    //      千万不要把 clk_sys 和 clk_pixel 放进同一组去"顺带"覆盖它。
    reg [1:0] cfg_ok_s, px_seen_u, any_ack_s;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin
            cfg_ok_s <= 2'b00; px_seen_u <= 2'b00; any_ack_s <= 2'b00;
        end else begin
            // ★ 2026-10-16：bit4 只看 cfg_done（374 条 ROM 走完了没有），
            //   不再和 ack_ok 相与 —— 原来那条"SCCB=0"就是因为 ack_ok 是
            //   **粘性**（1496 个字节挂 1 个 NACK 就永久为 0），把
            //   "配置完成"和"每条都 ACK"混成了一个位，导致一灭就说不清。
            //   ack_ok 交给 led[2]，any_ack 另给状态字 bit6。
            cfg_ok_s  <= {cfg_ok_s[0],  cfg_done};
            any_ack_s <= {any_ack_s[0], any_ack};
            px_seen_u <= {px_seen_u[0], px_seen_p};
        end
    end

    //   帧头里的状态字（2 位 hex，上位机脚本会翻译成人话）：
    //     bit0 pll_locked（hdmi_pll）      bit1 ddr_pll_lock
    //     bit2 cal_done                    bit3 cal_pass
    //     bit4 cfg_done（374 条 ROM 已走完）bit5 DVP 有像素
    //     bit6 any_ack（SCCB 至少收到过 1 个 ACK；=0 表示传感器根本没应答）
    //     bit7 恒 1 —— 「盒式平均缩略图」版标记，用来确认板上烧的是新版
    wire [7:0] stat = { 1'b1,               // bit7  版本标记
                        any_ack_s[1],       // bit6  SCCB 有过应答
                        px_seen_u[1],       // bit5  DVP 有像素
                        cfg_ok_s[1],        // bit4  cfg_done
                        cal_pass_lat,       // bit3
                        cal_done_lat,       // bit2
                        ddr_pll_lock,       // bit1
                        pll_locked };       // bit0

    // ========================================================
    //  13.0a ★★★ 2026-10-18 接手后新增：帧周期/行周期/显示帧周期/保持数遥测
    // ========================================================
    //  【为什么加这个】
    //    交接文档判断"画面抽动 = 传感器只有 15fps"（R1），但第一轮实测
    //    cam_period = 1,865,664（clk_sys 96MHz 拍）= 19.43ms = 51.46fps，
    //    而且 1,865,664 正好 = HTS(1896) × VTS(984) —— 暗示 PCLK 可能真是
    //    96MHz。为排除"done_toggle 每帧翻多次"或"sys_cnt 域不对"，
    //    本轮同时量：行周期、显示帧周期、相机帧之间的显示帧数（保持数）。
    //
    //  【判据】
    //    line_period ≈ 1896  -> PCLK = 96MHz；≈7584 -> PCLK = 24MHz
    //    disp_period ≈ 1.60M -> clk_sys 确实是 96MHz（60Hz 显示）
    //    hold = 相机一帧跨过几个显示帧：~1/2 = 51fps 源；~4/5 = 15fps 源
    //    frame_period / line_period 应 ≈ VTS = 984（自洽性）
    // --------------------------------------------------------
    reg [31:0] sys_cnt;
    reg [31:0] cam_period, line_period, disp_period;
    reg [31:0] sys_cnt_d, sys_cnt_l, sys_cnt_p;

    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) sys_cnt <= 32'd0;
        else          sys_cnt <= sys_cnt + 32'd1;
    end

    // ---- 相机帧周期：done_toggle（cmos_pclk）-> clk_sys ----
    reg        dt_s1, dt_s2, dt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin dt_s1 <= 1'b0; dt_s2 <= 1'b0; dt_d <= 1'b0; end
        else begin dt_s1 <= done_toggle; dt_s2 <= dt_s1; dt_d <= dt_s2; end
    end
    wire dt_evt = dt_s2 ^ dt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin cam_period <= 32'd0; sys_cnt_d <= 32'd0; end
        else if (dt_evt) begin cam_period <= sys_cnt - sys_cnt_d; sys_cnt_d <= sys_cnt; end
    end

    // ---- 行周期：line_end（cmos_pclk）-> clk_sys ----
    reg line_tgl;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) line_tgl <= 1'b0;
        else if (line_end) line_tgl <= ~line_tgl;
    end
    reg lt_s1, lt_s2, lt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin lt_s1 <= 1'b0; lt_s2 <= 1'b0; lt_d <= 1'b0; end
        else begin lt_s1 <= line_tgl; lt_s2 <= lt_s1; lt_d <= lt_s2; end
    end
    wire lt_evt = lt_s2 ^ lt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin line_period <= 32'd0; sys_cnt_l <= 32'd0; end
        else if (lt_evt) begin line_period <= sys_cnt - sys_cnt_l; sys_cnt_l <= sys_cnt; end
    end

    // ---- 显示帧周期：disp_frame_start（clk_pixel）-> clk_sys ----
    reg disp_tgl;
    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d)          disp_tgl <= 1'b0;
        else if (disp_frame_start) disp_tgl <= ~disp_tgl;
    end
    reg dpt_s1, dpt_s2, dpt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin dpt_s1 <= 1'b0; dpt_s2 <= 1'b0; dpt_d <= 1'b0; end
        else begin dpt_s1 <= disp_tgl; dpt_s2 <= dpt_s1; dpt_d <= dpt_s2; end
    end
    wire dpt_evt = dpt_s2 ^ dpt_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin disp_period <= 32'd0; sys_cnt_p <= 32'd0; end
        else if (dpt_evt) begin disp_period <= sys_cnt - sys_cnt_p; sys_cnt_p <= sys_cnt; end
    end

    // ---- 保持数：相机两帧之间跨过几个显示帧（全在 clk_pixel 域算）----
    //    done_event 与 disp_frame_start 都在 clk_pixel，直接数即可。
    //    hold_lat 稳定 ≥1 个相机帧周期（~19ms），用 toggle 握手送 clk_sys 安全。
    reg [7:0] pix_frm_cnt, pix_frm_d, hold_lat;
    reg       hold_tgl;
    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) begin
            pix_frm_cnt <= 8'd0; pix_frm_d <= 8'd0;
            hold_lat <= 8'd0;    hold_tgl <= 1'b0;
        end else begin
            if (disp_frame_start) pix_frm_cnt <= pix_frm_cnt + 8'd1;
            if (done_event) begin
                hold_lat <= pix_frm_cnt - pix_frm_d;
                pix_frm_d <= pix_frm_cnt;
                hold_tgl <= ~hold_tgl;
            end
        end
    end
    reg ht_s1, ht_s2, ht_d;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin ht_s1 <= 1'b0; ht_s2 <= 1'b0; ht_d <= 1'b0; end
        else begin ht_s1 <= hold_tgl; ht_s2 <= ht_s1; ht_d <= ht_s2; end
    end
    wire ht_evt = ht_s2 ^ ht_d;
    reg [7:0] hold_s;
    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s)     hold_s <= 8'd0;
        else if (ht_evt)  hold_s <= hold_lat;
    end

    // ========================================================
    //  13.0b ★★★ 第 35 轮（2026-10-16 20:xx）：缩略图"快照闸门"握手
    // ========================================================
    //  【为什么必须要有这个东西 —— 这是前几轮"画面撕裂"结论的假象来源】
    //
    //  缩略图 BRAM（ssram，64x48）是**单缓冲**：
    //    * 写侧 clk_pixel：每 10 个显示行写出一个块行，480 行写完一整幅
    //      ⇒ **每 16.7 ms（60 Hz）就把整幅 64x48 全部重写一遍**；
    //    * 读侧 clk_sys ：串口一帧 6247 字符 @115200 ≈ **0.54 s**。
    //
    //  0.54 s / 16.7 ms ≈ **32 个显示帧**。也就是说：串口在第 r 行时读到的
    //  单元，早被刷新了 32·(r/48) 次 —— **第 0 行是 0.53 s 前的画面，
    //  第 47 行才是当前画面**。传回来的根本不是一张图，而是
    //  **32 个不同瞬间拼成的"横向条带拼贴"**。
    //
    //  这**恰好**解释了此前测到的一切：
    //    · 行 内（同一瞬间、同一行）相关高达 +0.95  ← 每行确实是完整一行
    //    · 相邻行（相隔 ~11 ms、手持 + 自动曝光在跳）相关 ≈ 0
    //    · 相邻行"最佳水平位移"散乱、最佳相关也只有 0.34
    //    · 逐帧 58% 像素在变、行均值出现周期性暗带（自动曝光来回跳）
    //  ⇒ **第 32/33/34 轮的"撕裂 / 纵向错行"都是在测工具，不是在测画面。**
    //    （第 33 轮修 `wframe_vsync` 窄脉冲那件事本身是对的、保留；
    //      但它未必是当时那个现象的原因。）
    //
    //  【对策】给缩略图加"快照闸门"：
    //    ① clk_sys 要发新的一帧之前，先把 `cap_req` 拉高；
    //    ② clk_pixel 在**下一个显示帧起始**开窗（cap_en=1）；
    //    ③ 抓满 48 个块行（scy 从 47 回 0）立刻关窗并翻转 `cap_done_t`；
    //    ④ clk_sys 看到翻转才真正开始发帧 —— 整个 0.54 s 发送期间
    //       写侧是**冻结**的，BRAM 里始终是那一帧的快照。
    //    代价：每帧多等 ~20 ms（一帧抓取时间），帧率 1.5 fps 基本不变。
    //    资源：0 块 RAM（只是加了几根闸门线）。
    reg         cap_req;        // clk_sys -> clk_pixel：请抓一帧（电平）
    reg         cap_ready;      // 1 = 快照已就绪，可以开始发
    reg         cap_req_s1, cap_req_s2;   // cap_req 在 clk_pixel 域两级同步
    reg         cap_en;                   // clk_pixel：1 = 本显示帧正在抓
    reg         cap_done_t;               // clk_pixel -> clk_sys：抓满后翻转
    reg         cap_done_s1, cap_done_s2, cap_done_d;
    wire        cap_done_evt = cap_done_s2 ^ cap_done_d;

    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin
            cap_done_s1 <= 1'b0; cap_done_s2 <= 1'b0; cap_done_d <= 1'b0;
        end else begin
            cap_done_s1 <= cap_done_t;
            cap_done_s2 <= cap_done_s1;
            cap_done_d  <= cap_done_s2;
        end
    end

    // ---- 13.1 缩略图 BRAM：写侧（clk_pixel，10x10 盒式平均）----
    //  ★ 2026-10-16 19:2x 改版：原来是"每 10 像素 / 每 10 行**抽 1 个点**"。
    //    对 640x480 来说那是 10 倍**点采样**降采样 —— 严重混叠。而本工程
    //    存储路径上还带着 **4x4 Bayer 抖动**（CDITH）和 RGB332 量化，
    //    这些高频纹理和 10 倍抽样一打拍，缩略图上就退化成"一片雪花"，
    //    **根本判不了画面好坏**（噪点、摩尔纹、还是真坏了，分不出来）。
    //    改成对 10x10 方块**求和再取平均**（盒式滤波）：抖动被平均掉，
    //    混叠消失，缩略图终于等同于"缩小后的真实画面"。
    //
    //  数值账本：方块 = 10 个显示列 x 10 个显示行。显示是 2 倍最近邻放大，
    //    所以 10 显示列 = 5 个源像素（各出现 2 次），10 显示行 = 5 个源行。
    //      hacc = 本显示行 10 个像素之和        <= 10 *255 = 2550
    //      rsum = 本显示行的 64 个横向和
    //      vacc = 10 个显示行的 rsum 累加       <= 100*255 = 25500
    //      写出值 = vacc >> 7 = 0.78 x 方块平均灰度（线性、不截顶，
    //               因为 0.78*255 = 199 < 255）
    //    上位机只要把灰度乘 1.28 就还原（uart_view.py 已按此处理）。
    //
    //  ★ 结构说明：vacc / rsum 都是"每个元素各自独立"的寄存器组，
    //    写的时候索引是**编译期常量**（for 循环展开）→ 每位最多 1 个写口，
    //    不会退化成多写口 RAM。只有"写出到 BRAM"那一相是变址的。
    (* ramstyle = "block" *) reg [7:0] ssram [0:3071];
    reg  [11:0] ss_waddr;
    reg  [7:0]  ss_wdata;
    reg         ss_we;

    reg  [3:0]  h10;      // 行内像素计数 0..9
    reg  [5:0]  scx;      // 当前块列号 0..63
    reg  [3:0]  v10;      // 行计数 0..9
    reg  [5:0]  scy;      // 当前块行号 0..47
    reg         de_d;

    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) de_d <= 1'b0;
        else          de_d <= de_disp;
    end
    wire dline_end = de_d & ~de_disp;     // 显示行末脉冲（帧末也会有一拍）
                                          // ⚠️ 不能叫 line_end —— 第 570 行
                                          //    capture 域已有一个同名 wire

    reg  [11:0] hacc;                 // 横向累加
    reg  [11:0] rsum [0:63];          // 本显示行的 64 个横向和
    reg  [15:0] vacc [0:63];          // 10 行纵向累加
    reg  [5:0]  wo;                   // 写出扫描列号 0..63
    reg         wbusy;                // 1 = 正在把 vacc 写进 BRAM
    integer     kk;

    // ========================================================
    // 13.1a ★★★ 第 41 轮（2026-10-17）：1:1 原始像素窗口
    // --------------------------------------------------------
    //  为什么加这个：10x10 盒式平均 = **5:1 抽点**。被摄物只要有
    //  接近 5 像素周期的细结构（Bayer 抖动、物体纹理、传感器列
    //  固定图样），抽点就会拍频成"假周期"——我这轮先后被
    //  "横向周期 13 列 / 纵向周期 3 行"骗了两次。
    //
    //  做法：THUMB_RAW=1 时，ssram 里存的**不再是缩略图**，而是
    //  一幅 64 列 x 48 行 的 **1:1 原始字节**（直接取 fb_rdata，
    //  不做任何平均/抽点）。窗口取画面正中央：
    //      源像素 列 128..191 / 行 96..143  （fb 是 320x240）
    //  显示是 2 倍最近邻放大，所以显示坐标 = 源坐标 x2：
    //      显示 列 256..383 / 行 192..287，取偶数列/偶数行各一次。
    //
    //  判据（1:1 上的硬指标）：
    //    · 自然图像 -> 相邻像素 Pearson > 0.9，肉眼可见物体轮廓
    //    · 字节流坏了 -> 会看到周期 64/128/1024 的规则图样，或逐行
    //      错位的斜纹（这些在 5:1 抽点下是看不见的）
    //
    //  ★ 第 44 轮（10-17 17:57）：用户要求"看全视野"，置 0。
    //    1 = 中央 20%x20% 的 1:1 原始字节（字节级诊断用）
    //    0 = 全幅 64x48（fb 上 5x5 盒式平均，覆盖整个 4:3 视野）
    localparam THUMB_RAW = 1'b0;

    wire [8:0] raw_cx = hcnt[9:1] - 9'd128;   // 窗口内列号 0..63
    wire [8:0] raw_cy = vcnt[9:1] - 9'd96;    // 窗口内行号 0..47

    wire raw_colok = de_disp & (hcnt >= 10'd256) & (hcnt <= 10'd382) & ~hcnt[0];
    wire raw_rowok = (vcnt >= 10'd192) & (vcnt <= 10'd286) & ~vcnt[0];
    wire raw_hit   = cap_en & raw_colok & raw_rowok;

    reg        raw_we;
    reg [11:0] raw_wa;
    reg [7:0]  raw_wd;

    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) begin
            raw_we <= 1'b0; raw_wa <= 12'd0; raw_wd <= 8'd0;
        end else begin
            raw_we <= 1'b0;
            if (raw_hit) begin
                raw_we <= 1'b1;
                raw_wa <= {raw_cy[5:0], raw_cx[5:0]};   // ry*64 + rx
                raw_wd <= fb_rdata;
            end
        end
    end

    // 窗口最后一个字（ry=47, rx=63）抓完 -> 冻结
    wire raw_last = raw_hit & (raw_cy[5:0] == 6'd47) & (raw_cx[5:0] == 6'd63);

    // ---- ★★ 第 35 轮：抓帧窗口（见 13.0b 的长注释）----
    //   cap_row_done：第 48 个块行（scy==47）扫描写满的那一拍。
    //   wbusy && wo==63 && scy==47  → 整幅 64x48 刚好写完，此刻冻结最干净。
    wire cap_row_done = wbusy && (wo == 6'd63) && (scy == 6'd47);

    // 两种模式共用同一套握手
    wire cap_done_now = THUMB_RAW ? raw_last : cap_row_done;

    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) begin
            cap_req_s1 <= 1'b0; cap_req_s2 <= 1'b0;
            cap_en     <= 1'b0; cap_done_t <= 1'b0;
        end else begin
            cap_req_s1 <= cap_req;            // 电平信号，两级同步足够
            cap_req_s2 <= cap_req_s1;
            // 只在显示帧起始开窗 —— 保证 48 个块行全部来自同一个显示帧
            if (disp_frame_start) cap_en <= cap_req_s2;
            if (cap_done_now) begin
                cap_en     <= 1'b0;           // 写满即冻结，发送期间不动
                cap_done_t <= ~cap_done_t;    // 通知 clk_sys：快照好了
            end
        end
    end

    always @(posedge clk_pixel or negedge rst_n_d) begin
        if (!rst_n_d) begin
            h10 <= 4'd0;  scx <= 6'd0;  hacc <= 12'd0;
            v10 <= 4'd0;  scy <= 6'd0;  wo <= 6'd0;  wbusy <= 1'b0;
            ss_we <= 1'b0; ss_waddr <= 12'd0; ss_wdata <= 8'd0;
            for (kk = 0; kk < 64; kk = kk + 1) begin
                rsum[kk] <= 12'd0;
                vacc[kk] <= 16'd0;
            end
        end else begin
            ss_we <= 1'b0;

            if (!cap_en) begin
                // ---- ★★ 冷冻相（第 35 轮）----
                //  没在抓帧就把所有计数器摁回 0：BRAM 里保持整帧快照不动，
                //  下次开窗一定从"块行 0 / 列 0 / 第一行像素"干净起步。
                //  （vacc 不用清 —— 抓满时写出口那支已经清过了。）
                h10 <= 4'd0;  scx <= 6'd0;  hacc <= 12'd0;
                v10 <= 4'd0;  scy <= 6'd0;  wo   <= 6'd0;  wbusy <= 1'b0;
            end else if (wbusy) begin
                // ---- 写出相：64 个块列，每拍写一个到 BRAM ----
                ss_we    <= 1'b1;
                ss_waddr <= {scy, wo};          // scy*64 + wo
                ss_wdata <= vacc[wo][14:7];     // 0.78 x 方块均值
                if (wo == 6'd63) begin
                    wo    <= 6'd0;
                    wbusy <= 1'b0;
                    scy   <= (scy == 6'd47) ? 6'd0 : (scy + 6'd1);
                    for (kk = 0; kk < 64; kk = kk + 1) vacc[kk] <= 16'd0;
                end else begin
                    wo <= wo + 6'd1;
                end
            end else begin
                // ---- 累加相 ----
                if (de_disp) begin
                    if (h10 == 4'd9) begin
                        // hacc 已含前 9 个像素，再加当前这拍 = 完整 10 个
                        rsum[scx] <= {4'b0, hacc} + {4'b0, fb_rdata};
                        hacc      <= 12'd0;
                        h10       <= 4'd0;
                        scx       <= (scx == 6'd63) ? 6'd0 : (scx + 6'd1);
                    end else begin
                        h10  <= h10 + 4'd1;
                        hacc <= hacc + {4'b0, fb_rdata};
                    end
                end else begin
                    h10  <= 4'd0;      // 行末/消隐归零，保证每行列对齐
                    scx  <= 6'd0;
                    hacc <= 12'd0;
                end

                // 每个显示行结束：把本行的 64 个横向和并进 vacc
                if (dline_end) begin
                    for (kk = 0; kk < 64; kk = kk + 1)
                        vacc[kk] <= vacc[kk] + rsum[kk];
                    if (v10 == 4'd9) begin
                        v10   <= 4'd0;
                        wo    <= 6'd0;
                        wbusy <= 1'b1;         // 10 行到齐 -> 触发写出
                    end else begin
                        v10 <= v10 + 4'd1;
                    end
                end
            end
        end
    end

    // BRAM 写口：两种模式二选一（第 41 轮的 1:1 窗口 / 原来的盒式缩略图）
    wire        ssram_we    = THUMB_RAW ? raw_we    : ss_we;
    wire [11:0] ssram_waddr = THUMB_RAW ? raw_wa    : ss_waddr;
    wire [7:0]  ssram_wdata = THUMB_RAW ? raw_wd    : ss_wdata;

    always @(posedge clk_pixel) if (ssram_we) ssram[ssram_waddr] <= ssram_wdata;

    // ---- 13.1b 缩略图 BRAM：读侧（clk_sys）----
    reg  [11:0] ss_raddr;
    reg  [7:0]  ss_rdata;
    always @(posedge clk_sys) ss_rdata <= ssram[ss_raddr];

    // ---- 13.2 UART 发送器：115200 @ 96 MHz -> 分频 833 ----
    //   96e6 / 833 = 115246 bps，误差 +0.04%，任何 USB-UART 都能吃。
    //   帧格式：1 start(0) + 8 data(LSB first) + 1 stop(1) = 10 bit。
    //   ★★ 第 28 轮修 bug：原来 bbit==9 就收尾，等于 **stop 位只维持 1 个
    //      时钟（~10 ns）** 就被下一个 start 打断 —— 上位机在 stop 位中点
    //      采样必然读到 0，整帧报 framing error。改成 bbit 走到 10（= 一个
    //      完整分频周期）才释放 bbusy，stop 位走满 833 拍。
    localparam [9:0] BAUD_DIV = 10'd833;

    reg  [9:0]  bdiv;
    reg  [3:0]  bbit;
    reg  [9:0]  bsh;          // {stop=1, data[7:0], start=0}
    reg         txd_r;
    reg         bbusy;
    reg         tx_ack;       // 单周期脉冲："这个字符我收下了"
    reg         tx_req;       // ★ 电平请求：保持到被 ack —— 不是单周期脉冲
    reg  [7:0]  tx_data;

    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin
            bdiv <= 10'd0; bbit <= 4'd0; bsh <= 10'h3FF;
            txd_r <= 1'b1; bbusy <= 1'b0; tx_ack <= 1'b0;
        end else begin
            tx_ack <= 1'b0;
            if (!bbusy) begin
                txd_r <= 1'b1;                       // 空闲线 = 高
                if (tx_req) begin
                    bsh    <= {1'b1, tx_data, 1'b0};
                    txd_r  <= 1'b0;                  // start bit 立刻拉低
                    bbit   <= 4'd1;
                    bdiv   <= 10'd0;
                    bbusy  <= 1'b1;
                    tx_ack <= 1'b1;                  // 通知发送状态机
                end
            end else if (bdiv == BAUD_DIV - 10'd1) begin
                bdiv <= 10'd0;
                if (bbit == 4'd10) begin
                    txd_r <= 1'b1;                   // stop 位已走满一整 bit
                    bbusy <= 1'b0;
                end else begin
                    txd_r <= bsh[bbit];
                    bbit  <= bbit + 4'd1;
                end
            end else begin
                bdiv <= bdiv + 10'd1;
            end
        end
    end

    assign fpga_txd = txd_r;

    // ---- 13.3 发送状态机（clk_sys 域）----
    //   0 IDLE    等间隔 -> 发 '@'
    //   1..2      发帧号（2 位 hex）
    //   3..4      发状态字（2 位 hex）
    //   16..23    发相机帧周期（8 位 hex）★ 2026-10-18 新增
    //   24..29    发行周期（6 位 hex）
    //   30..35    发显示帧周期（6 位 hex）
    //   36..37    发保持数 hold（2 位 hex）
    //   5..6      发 CR LF
    //   7 PREP    设 BRAM 读地址
    //   8 WAIT    等 BRAM 同步读延迟
    //   9 LATCH   锁存像素
    //   10..11    发该像素的高/低 nibble
    //   12        行末 -> 发 CR
    //   13        发 LF
    //   14        推进像素 / 整帧结束回 IDLE
    //   15 WAITCAP ★第 35 轮新增：等 clk_pixel 抓完一帧快照（cap_ready）
    //
    //   ★★ 第 28 轮修 bug（致命）：请求原来是**单周期脉冲**，而 UART 只在
    //      !bbusy 时才看 tx_req —— 于是发完第一个字符后 UART 忙 8330 拍，
    //      请求被丢掉，状态机**永久死锁**；整块板上电只发出过一个 '@'。
    //      现在 tx_req 是**电平**，一直举着直到 UART 用 tx_ack 收下。
    //      这是标准 valid/ready 握手。
    reg  [5:0]  ust;
    reg         upend;        // "字符已请求，正在等 UART 接受"
    reg  [11:0] upix;         // 0..3071
    reg  [7:0]  ubyte;
    reg  [7:0]  ufnum;        // 帧号
    reg  [26:0] ugap;

    //   帧间空隙 ~104 ms @ 96 MHz：一帧 6277 字符 ≈ 0.54 s，
    //   所以约 0.65 s 一帧（≈1.5 fps），串口上几乎一直有数据 ——
    //   上位机任何时候连上，最多等 0.1 s 就能抓到帧头。
    localparam [26:0] GAP_MAX = 27'd10_000_000;

    function [7:0] hexch;
        input [3:0] v;
        begin
            hexch = (v < 4'd10) ? (8'h30 + {4'd0, v})    // '0'..'9'
                                : (8'h37 + {4'd0, v});   // 'A'..'F'
        end
    endfunction

    always @(posedge clk_sys or negedge rst_n_s) begin
        if (!rst_n_s) begin
            ust <= 4'd0; upend <= 1'b0; tx_req <= 1'b0; tx_data <= 8'hFF;
            upix <= 12'd0; ubyte <= 8'd0; ufnum <= 8'd0; ugap <= 27'd0;
            ss_raddr <= 12'd0;
            cap_req <= 1'b0; cap_ready <= 1'b0;
        end else begin
            if (cap_done_evt) cap_ready <= 1'b1;   // ★ 第 35 轮：快照好了
            if (upend) begin
                // ★ 请求是电平：举着不动，直到 UART 收下（tx_ack）。
                //   原来这里每拍都 `tx_req <= 1'b0`，把请求缩成了 1 拍脉冲
                //   —— 这就是"发不出数据"的根因。
                if (tx_ack) begin
                    upend  <= 1'b0;
                    tx_req <= 1'b0;
                end
            end else begin
                case (ust)
                4'd0:  if (ugap >= GAP_MAX) begin
                           // ★★ 第 35 轮：不要马上发！
                           //   先请 clk_pixel 抓一帧干净快照（cap_req），
                           //   等它抓完 48 行回 cap_ready，再开始发。
                           //   不这么做的话，发送要花 0.54 s = 32 个显示帧，
                           //   而 BRAM 每 16.7 ms 就被整幅重写一遍
                           //   ⇒ 发出去的是"32 个瞬间的横向拼贴"，
                           //     纵向永远对不上（前几轮"撕裂"假象的真因）。
                           cap_req <= 1'b1;
                           ugap    <= 27'd0;
                           ust     <= 4'd15;
                       end else begin
                           ugap <= ugap + 27'd1;
                       end
                4'd15: if (cap_ready) begin
                           // 快照就绪：放掉请求，正式开始发这一帧
                           cap_req   <= 1'b0;
                           cap_ready <= 1'b0;
                           tx_data <= 8'h40;          // '@'
                           tx_req  <= 1'b1; upend <= 1'b1;
                           upix <= 12'd0; ust <= 4'd1;
                       end
                4'd1:  begin tx_data <= hexch(ufnum[7:4]); tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd2;  end
                4'd2:  begin tx_data <= hexch(ufnum[3:0]); tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd3;  end
                4'd3:  begin tx_data <= hexch(stat[7:4]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd4;  end
                4'd4:  begin tx_data <= hexch(stat[3:0]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd16; end
                // ---- ★ 2026-10-18 v2：帧周期/行周期/显示帧周期/保持数 ----
                6'd16: begin tx_data <= hexch(cam_period[31:28]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd17; end
                6'd17: begin tx_data <= hexch(cam_period[27:24]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd18; end
                6'd18: begin tx_data <= hexch(cam_period[23:20]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd19; end
                6'd19: begin tx_data <= hexch(cam_period[19:16]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd20; end
                6'd20: begin tx_data <= hexch(cam_period[15:12]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd21; end
                6'd21: begin tx_data <= hexch(cam_period[11:8]);   tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd22; end
                6'd22: begin tx_data <= hexch(cam_period[7:4]);    tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd23; end
                6'd23: begin tx_data <= hexch(cam_period[3:0]);    tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd24; end
                6'd24: begin tx_data <= hexch(line_period[23:20]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd25; end
                6'd25: begin tx_data <= hexch(line_period[19:16]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd26; end
                6'd26: begin tx_data <= hexch(line_period[15:12]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd27; end
                6'd27: begin tx_data <= hexch(line_period[11:8]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd28; end
                6'd28: begin tx_data <= hexch(line_period[7:4]);   tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd29; end
                6'd29: begin tx_data <= hexch(line_period[3:0]);   tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd30; end
                6'd30: begin tx_data <= hexch(disp_period[23:20]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd31; end
                6'd31: begin tx_data <= hexch(disp_period[19:16]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd32; end
                6'd32: begin tx_data <= hexch(disp_period[15:12]); tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd33; end
                6'd33: begin tx_data <= hexch(disp_period[11:8]);  tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd34; end
                6'd34: begin tx_data <= hexch(disp_period[7:4]);   tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd35; end
                6'd35: begin tx_data <= hexch(disp_period[3:0]);   tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd36; end
                6'd36: begin tx_data <= hexch(hold_s[7:4]);        tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd37; end
                6'd37: begin tx_data <= hexch(hold_s[3:0]);        tx_req <= 1'b1; upend <= 1'b1; ust <= 6'd5;  end
                4'd5:  begin tx_data <= 8'h0D;             tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd6;  end
                4'd6:  begin tx_data <= 8'h0A;             tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd7;  end
                4'd7:  begin ss_raddr <= upix; ust <= 4'd8;  end            // PREP
                4'd8:  begin ust <= 4'd9; end                               // WAIT
                4'd9:  begin ubyte <= ss_rdata; ust <= 4'd10; end           // LATCH
                4'd10: begin tx_data <= hexch(ubyte[7:4]); tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd11; end
                4'd11: begin tx_data <= hexch(ubyte[3:0]); tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd12; end
                4'd12: if (upix[5:0] == 6'd63) begin
                           tx_data <= 8'h0D; tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd13;
                       end else begin
                           upix <= upix + 12'd1; ust <= 4'd7;
                       end
                4'd13: begin tx_data <= 8'h0A; tx_req <= 1'b1; upend <= 1'b1; ust <= 4'd14; end
                4'd14: if (upix == 12'd3071) begin
                           ufnum <= ufnum + 8'd1; ugap <= 27'd0; ust <= 4'd0;
                       end else begin
                           upix <= upix + 12'd1; ust <= 4'd7;
                       end
                default: ust <= 4'd0;
                endcase
            end
        end
    end

endmodule
