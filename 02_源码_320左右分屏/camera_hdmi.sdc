# ============================================================
#  camera_hdmi.sdc
#  OV5640 DVP + HDMI TMDS 的时序约束
#
#  这个文件是"用户 SDC"，即 efx_pnr 用 --sdc_file 指定的那一个。
#
#  ★★ 2026-10-16 实测修正（重要）：
#     本文件原来写着"会和 PT 自动生成的 outflow/camera_hdmi.pt.sdc
#     合并后再交给 efx_pnr" —— **这个假设是错的，实测不成立**。
#     证据：整条构建链跑完后，outflow/camera_hdmi.timing.rpt 的
#           "Clock Frequency Summary" 里只有 clk24m 和 cmos_pclk 两个时钟；
#           而 PT 产物 pt.sdc 里明明有 7 条 create_clock。
#     对照官方 02-1 工程：它的 timing.rpt 里 clk_sys/clk_pixel/
#     clk_pixel_10x/tdqss/core/tac/twd 全部在列 —— 因为官方 XML 是把 PT 产物
#     Ti60_Demo.pt.sdc **直接登记为工程 SDC**（<efx:sdc_file name="..."/>），
#     而我们登记的是本文件。
#     结论：**本文件必须自己把 7 个时钟全部 create 出来**，否则
#     clk_sys / clk_pixel / clk_pixel_10x 和 DDR 四个时钟全部处于
#     "无约束 = 不做时序分析"状态 —— 不报违例 ≠ 满足时序，是盲飞。
# ============================================================
#  时钟一览
#      clk24m        41.667 ns   板载 24 MHz 晶振 (ball C5)，SCCB/I2C 用
#      cmos_pclk     10.417 ns   传感器像素时钟，取 96 MHz 上限做约束
#      clk_sys       10.417 ns   PLL CLKOUT0 = 96.0 MHz
#      clk_pixel     39.683 ns   PLL CLKOUT1 = 25.2 MHz
#      clk_pixel_10x  7.937 ns   PLL CLKOUT2 = 126.0 MHz
# ============================================================

# ---------- 时钟定义（本文件自己给全，共 9 个） ----------

# 板级 24MHz 晶振 + 传感器像素时钟
#   （PT 只留了 cmos_pclk 的 <USER_PERIOD> 占位符，必须用户补）
create_clock -name clk24m    -period 41.667 [get_ports {clk24m}]
create_clock -name cmos_pclk -period 10.417 [get_ports {cmos_pclk}]

# hdmi_pll（PLL_TL0，参考 clk24m）的三个输出
create_clock -name clk_sys   -period 10.417 [get_ports {clk_sys}]
create_clock -name clk_pixel -period 39.683 [get_ports {clk_pixel}]
#   clk_pixel_10x：LVDS 串行时钟，peri.xml 里 phase_setting=2（22.5 度）。
#   180 度相位时波形是 {0.496 4.465}（= 边沿相对周期起点的偏移）。
create_clock -name clk_pixel_10x -waveform {0.496 4.465} -period 7.937 [get_ports {clk_pixel_10x}]

# ddr_pll（PLL_BR0，参考 clk_sys 96MHz，M/N/O = 2/1/1 → VCO 3072MHz）
#   四路输出 out_divider = 8 / 16 / 8 / 8  ->  384 / 192 / 384 / 384 MHz
#   twd_clk 的 phase_setting=4（4/16 周期 = 0.651 ns）
create_clock -name tdqss_clk -period 2.604 [get_ports {tdqss_clk}]
create_clock -name core_clk  -period 5.208 [get_ports {core_clk}]
create_clock -name tac_clk   -period 2.604 [get_ports {tac_clk}]
create_clock -name twd_clk   -waveform {0.651 1.953} -period 2.604 [get_ports {twd_clk}]

# ---------- 异步时钟组 ----------
# clk24m / cmos_pclk 都是外部自由振荡源；
# clk_sys / clk_pixel / clk_pixel_10x 同源于 hdmi_pll 的 VCO，彼此同步，
# 所以放在同一组里，让 PT 正常做跨域分析。
# 三个 PLL 输出之间确实有 1 个真实跨域点：
#     sys_cnt[23] (clk_sys) -> syshb_s (clk_pixel)   仅驱动 LED
# 因为源是极慢的翻转位、目的端又是两级同步器，时序天然宽松。
set_clock_groups -asynchronous \
    -group {clk24m} \
    -group {cmos_pclk} \
    -group {clk_sys clk_pixel clk_pixel_10x}

# ---------- clk_sys <-> clk_pixel：显式异步 ----------
#  （照抄官方 Ti60_Demo.pt.sdc 第 52-53 行，语句一字未改）
#
#  ★★ 2026-10-16 实测补入 —— 不补会真的报违例：
#     上面把 clk_sys / clk_pixel 放进**同一个** clock group，含义是
#     "两者同源于 hdmi_pll 的 VCO、互相关联"，于是工具会把这一对时钟
#     之间的所有路径都当**同步路径**去分析。但它们之间其实**只有 CDC
#     控制信号，没有同步数据路径**：
#         clk_sys   -> clk_pixel : axi_reset (ddr_rst_sync) -> R0_FIFO_8 rd_rst
#         clk_pixel -> clk_sys   : axi4_ctrl/rfifo_rst      -> R0_FIFO_8 wr_rst
#     这两条的终点都是双口 FIFO 的 "~FF|SR"（Set/Reset 脚），即 FIFO 内部
#     的**异步复位同步寄存器**（对 a_rst_i 打两拍），天生不可能满足
#     setup/recovery —— 必须声明为异步。
#     实测不加这两条时的违例（camera_hdmi.timing.rpt）：
#         clk_sys   -> clk_pixel   -1.319 ns
#         clk_pixel -> clk_sys     -0.864 ns
#     加了之后这一对时钟不再出现在违例表里（其余时钟不受影响）。
set_false_path -from [get_clocks {clk_sys}] -to [get_clocks {clk_pixel}]
set_false_path -from [get_clocks {clk_pixel}] -to [get_clocks {clk_sys}]

# ---------- DVP 输入，参考 cmos_pclk ----------
# 传感器在 PCLK 边沿发数据，板上走线有几 ns 偏斜
set_input_delay -clock cmos_pclk -max 4.000 [get_ports {cmos_d[*]}]
set_input_delay -clock cmos_pclk -min 1.000 [get_ports {cmos_d[*]}]
set_input_delay -clock cmos_pclk -max 4.000 [get_ports {cmos_href}]
set_input_delay -clock cmos_pclk -min 1.000 [get_ports {cmos_href}]
set_input_delay -clock cmos_pclk -max 4.000 [get_ports {cmos_vsync}]
set_input_delay -clock cmos_pclk -min 1.000 [get_ports {cmos_vsync}]

# ---------- clk24m 域的板级慢速控制脚 ----------
# -min 0.000 等于不设保持要求，避免路由器为满足"不可能的 hold"插延迟单元
set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_xclk}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_xclk}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_sclk}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_sclk}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_sdat_out}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_sdat_out}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_sdat_oe}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_sdat_oe}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_ctl1}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_ctl1}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_ctl2}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_ctl2}]

set_output_delay -clock clk24m -max 8.000 [get_ports {cmos_ctl3}]
set_output_delay -clock clk24m -min 0.000 [get_ports {cmos_ctl3}]

# ---------- SCCB 回读 (ACK) ----------
# sda_in 不是"被时钟同步的数据"：I2C 协议保证它在 SCL 高电平期间是稳定电平，
# 而 RTL 在 SCL 拉高后要数约 120 个 clk24m 周期才去采样它 —— 采样点距离
# 任何边沿都有好几微秒。所以这里刻意把 -min 抬到 2 ns：
# 既表达了"数据远晚于时钟边沿才变化"，又消掉了 -4 ps 的假 hold 违例。
set_input_delay -clock clk24m -max 20.000 [get_ports {cmos_sdat_in}]
set_input_delay -clock clk24m -min  2.000 [get_ports {cmos_sdat_in}]

# ---------- 放松项 ----------
# 状态灯只是给人看的
set_false_path -to   [get_ports {led[*]}]
# 异步复位输入
set_false_path -from [get_ports {rst_n}]

# ============================================================
#  DDR3 时钟关系
#  （照抄官方 Ti60_Demo.pt.sdc 第 35-45 行，语句一字未改）
#
#  背景：官方工程把 PT 产物 Ti60_Demo.pt.sdc 直接登记为工程 SDC，
#  其中手工增补了下面这几条。我们同样把它们放进本文件。
#  create_clock（tdqss/core/tac/twd）已在本文件上方"时钟定义"段自己写好
#  （原因见文件头：PT 产物实测不会被 efx_pnr 加载）。
#  这里只放"时钟之间的关系"，即 PT 不会生成的那一部分。
#
#  逐条理由：
#   * tdqss/core/tac/twd 来自 ddr_pll(PLL_BR0)，参考 clk_sys；
#     与 hdmi_pll 的三个输出虽同源但相位不确定 -> 不与 hdmi 时钟互测。
#   * clk_sys <-> core_clk 是【真实跨域路径】（axi4_ctrl 侧在 clk_sys，
#     DdrCtrl 侧在 core_clk），必须保留分析，故用 set_max_delay 4.5 ns
#     收紧（比 core_clk 周期 5.208 ns 更紧），不能写 false_path。
#   * tac_clk / twd_clk 只提供相位与驱动，无逻辑跨域 -> false_path。
#   * twd_clk -> core_clk 组内互不分析。
#
#  ⚠️ 官方同段里还有涉及 csi_rxc_i / dsi_byteclk_i / clk_lvds_1x /
#     clk_lvds_7x / clk_27m / clk_54m 的 false_path —— 那是官方 DSI/LVDS
#     屏通路的时钟，本工程没有，抄了会报找不到对象。DDR 这 8 条只抄这一部分。
#
#  ⚠️ 更正（2026-10-16 实测）：本文件上一版的注释曾写"本文件上面已有的
#       set_clock_groups -asynchronous -group {clk24m} -group {cmos_pclk}
#           -group {clk_sys clk_pixel clk_pixel_10x}
#     等效于官方 Ti60_Demo.pt.sdc 第 52-56 行的 4 条 false_path" —— **错的**。
#     那个 set_clock_groups 只隔离了 clk24m、cmos_pclk，把 clk_sys 和
#     clk_pixel 留在**同一组**里（故意，好让它们之间的路径被分析）；
#     它并不等价于 false_path。官方第 52-53 行那两条必须**单独补**，
#     已补到本文件上面"clk_sys <-> clk_pixel：显式异步"一节。
# ============================================================
set_clock_groups -group {tdqss_clk core_clk tac_clk twd_clk}

set_max_delay -from [get_clocks {core_clk}] -to [get_clocks {clk_sys}] 4.5
set_false_path -from [get_clocks {tac_clk}] -to [get_clocks {clk_sys}]
set_false_path -from [get_clocks {twd_clk}] -to [get_clocks {clk_sys}]

set_max_delay -from [get_clocks {clk_sys}] -to [get_clocks {core_clk}] 4.5
set_false_path -from [get_clocks {clk_sys}] -to [get_clocks {tac_clk}]
set_false_path -from [get_clocks {clk_sys}] -to [get_clocks {twd_clk}]

set_false_path -from [get_clocks twd_clk] -to [get_clocks core_clk]
