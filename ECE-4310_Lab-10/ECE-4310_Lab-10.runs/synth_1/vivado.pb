
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
create_project: 2

00:00:072

00:00:072

1458.5592
57.8362
4332
10502Z17-722h px� 
k
Command: %s
53*	vivadotcl2:
8synth_design -top mcs_top_sampler -part xc7a100tcsg324-1Z4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
{
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2

xc7a100tZ17-347h px� 
k
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2

xc7a100tZ17-349h px� 
E
Loading part %s157*device2
xc7a100tcsg324-1Z21-403h px� 
[
$Part: %s does not have CEAM library.966*device2
xc7a100tcsg324-1Z21-9227h px� 
o
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
4Z8-7079h px� 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px� 
O
#Helper process launched with PID %s4824*oasys2
141532Z8-7075h px� 
�
%s*synth2�
�Starting RTL Elaboration : Time (s): cpu = 00:00:05 ; elapsed = 00:00:06 . Memory (MB): peak = 2250.047 ; gain = 412.715 ; free physical = 403 ; free virtual = 9427
h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2	
pcm_out2
wire2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_adsr_core.sv2
478@Z8-11241h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2
q2
wire2f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/debounce_counter.sv2
238@Z8-11241h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2
rx_done_tick2
wire2_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart.sv2
318@Z8-11241h px� 
�
synthesizing module '%s'%s4497*oasys2
mcs_top_sampler2
 2e
a/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/mcs_top_sampler.sv2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
cpu2
 2X
T/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/synth/cpu.v2
538@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2	
bd_39142
 2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
108@Z8-6157h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_dlmb_02r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_3/synth/bd_3914_dlmb_0.vhd2
898@Z8-638h px� 
S
%s
*synth2;
9	Parameter C_LMB_NUM_SLAVES bound to: 2 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_EXT_RESET_HIGH bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
lmb_v102s
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
912
U02	
lmb_v102r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_3/synth/bd_3914_dlmb_0.vhd2
1658@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2	
lmb_v102u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1408@Z8-638h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
FDS2;
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2
411372

POR_FF_I2
FDS2u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1708@Z8-3491h px� 
�
synthesizing module '%s'%s4497*oasys2
FDS2
 2=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2	
411378@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
FDS2
 2
02
12=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2	
411378@Z8-6155h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2	
lmb_v102
02
12u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1408@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_dlmb_02
02
12r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_3/synth/bd_3914_dlmb_0.vhd2
898@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2	
LMB_Rst2
bd_3914_dlmb_02
dlmb2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
1468@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
dlmb2
bd_3914_dlmb_02
252
242a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
1468@Z8-7023h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_dlmb_cntlr_02x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/synth/bd_3914_dlmb_cntlr_0.vhd2
848@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
�
%s
*synth2h
f	Parameter C_HIGHADDR bound to: 64'b0000000000000000000000000000000000000000000000011111111111111111 
h p
x
� 
�
%s
*synth2h
f	Parameter C_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
L
%s
*synth24
2	Parameter C_NUM_LMB bound to: 1 - type: integer 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK1 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK2 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK3 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK4 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK5 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK6 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK7 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ARBITRATION bound to: 0 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_ECC bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_INTERCONNECT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FAULT_INJECT bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_CE_FAILING_REGISTERS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_UE_FAILING_REGISTERS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_ECC_STATUS_REGISTERS bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_ECC_ONOFF_REGISTER bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_ECC_ONOFF_RESET_VALUE bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_CE_COUNTER_WIDTH bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_WRITE_ACCESS bound to: 2 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_BRAM_AWIDTH bound to: 32 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_S_AXI_CTRL_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_S_AXI_CTRL_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
lmb_bram_if_cntlr2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
50372
U02
lmb_bram_if_cntlr2x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/synth/bd_3914_dlmb_cntlr_0.vhd2
2758@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
lmb_bram_if_cntlr2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
52198@Z8-638h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
%s
*synth2h
f	Parameter C_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK1 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK2 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK3 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK4 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK5 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK6 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK7 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_ECC_WIDTH bound to: 7 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_ECC bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_NUM_LMB bound to: 1 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ARBITRATION bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
lmb_mux2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
41432
	lmb_mux_I2	
lmb_mux2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
55458@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2	
lmb_mux2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
42888@Z8-638h px� 
H
%s
*synth20
.	Parameter C_AW bound to: 32 - type: integer 
h p
x
� 
{
%s
*synth2c
a	Parameter C_BAR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
pselect_mask2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34672
pselect_mask_lmb2
pselect_mask2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
44018@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2(
&lmb_bram_if_cntlr_v4_0_24_pselect_mask2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34828@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2(
&lmb_bram_if_cntlr_v4_0_24_pselect_mask2
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34828@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2	
lmb_mux2
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
42888@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
lmb_bram_if_cntlr2
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
52198@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_dlmb_cntlr_02
02
12x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/synth/bd_3914_dlmb_cntlr_0.vhd2
848@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_ilmb_02r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/synth/bd_3914_ilmb_0.vhd2
898@Z8-638h px� 
S
%s
*synth2;
9	Parameter C_LMB_NUM_SLAVES bound to: 1 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_EXT_RESET_HIGH bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
lmb_v102s
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
912
U02	
lmb_v102r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/synth/bd_3914_ilmb_0.vhd2
1658@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
lmb_v10__parameterized12u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1408@Z8-638h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
FDS2;
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2
411372

POR_FF_I2
FDS2u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1708@Z8-3491h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
lmb_v10__parameterized12
02
12u
q/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/hdl/lmb_v10_v3_0_vh_rfs.vhd2
1408@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_ilmb_02
02
12r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_2/synth/bd_3914_ilmb_0.vhd2
898@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2	
LMB_Rst2
bd_3914_ilmb_02
ilmb2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
1948@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
ilmb2
bd_3914_ilmb_02
252
242a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
1948@Z8-7023h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_ilmb_cntlr_02x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_5/synth/bd_3914_ilmb_cntlr_0.vhd2
848@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
�
%s
*synth2h
f	Parameter C_HIGHADDR bound to: 64'b0000000000000000000000000000000000000000000000011111111111111111 
h p
x
� 
�
%s
*synth2h
f	Parameter C_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
L
%s
*synth24
2	Parameter C_NUM_LMB bound to: 1 - type: integer 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000010000000000000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK1 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK2 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK3 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK4 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK5 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK6 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK7 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ARBITRATION bound to: 0 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_ECC bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_INTERCONNECT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FAULT_INJECT bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_CE_FAILING_REGISTERS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_UE_FAILING_REGISTERS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_ECC_STATUS_REGISTERS bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_ECC_ONOFF_REGISTER bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_ECC_ONOFF_RESET_VALUE bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_CE_COUNTER_WIDTH bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_WRITE_ACCESS bound to: 2 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_BRAM_AWIDTH bound to: 32 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_S_AXI_CTRL_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_S_AXI_CTRL_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
lmb_bram_if_cntlr2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
50372
U02
lmb_bram_if_cntlr2x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_5/synth/bd_3914_ilmb_cntlr_0.vhd2
2758@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2#
!lmb_bram_if_cntlr__parameterized12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
52198@Z8-638h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
%s
*synth2h
f	Parameter C_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000010000000000000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK1 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK2 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK3 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK4 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK5 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK6 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
}
%s
*synth2e
c	Parameter C_MASK7 bound to: 64'b0000000000000000000000000000000000000000100000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_ECC_WIDTH bound to: 7 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_ECC bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_NUM_LMB bound to: 1 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ARBITRATION bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
lmb_mux2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
41432
	lmb_mux_I2	
lmb_mux2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
55458@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
lmb_mux__parameterized12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
42888@Z8-638h px� 
H
%s
*synth20
.	Parameter C_AW bound to: 32 - type: integer 
h p
x
� 
{
%s
*synth2c
a	Parameter C_BAR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000010000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
pselect_mask2}
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34672
pselect_mask_lmb2
pselect_mask2
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
44018@Z8-3491h px� 
�
synthesizing module '%s'638*oasys28
6lmb_bram_if_cntlr_v4_0_24_pselect_mask__parameterized02
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34828@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys28
6lmb_bram_if_cntlr_v4_0_24_pselect_mask__parameterized02
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
34828@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
lmb_mux__parameterized12
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
42888@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2#
!lmb_bram_if_cntlr__parameterized12
02
12
{/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_4/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd2
52198@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_ilmb_cntlr_02
02
12x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_5/synth/bd_3914_ilmb_cntlr_0.vhd2
848@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_iomodule_0_02x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/synth/bd_3914_iomodule_0_0.vhd2
858@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FREQ bound to: 100000000 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_INSTANCE bound to: iomodule - type: string 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_AVOID_PRIMITIVES bound to: 0 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
�
%s
*synth2h
f	Parameter C_HIGHADDR bound to: 64'b0000000000000000000000000000000010000000000000001111111111111111 
h p
x
� 
�
%s
*synth2h
f	Parameter C_BASEADDR bound to: 64'b0000000000000000000000000000000010000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
�
%s
*synth2k
i	Parameter C_IO_HIGHADDR bound to: 64'b0000000000000000000000000000000011111111111111111111111111111111 
h p
x
� 
�
%s
*synth2k
i	Parameter C_IO_BASEADDR bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 

%s
*synth2g
e	Parameter C_IO_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_AWIDTH bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_LMB_DWIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_IO_BUS bound to: 1 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_RX bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_TX bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_BAUDRATE bound to: 9600 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_UART_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_USE_PARITY bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_ODD_PARITY bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_RX_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_TX_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_UART_ERROR_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_UART_PROG_BAUDRATE bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_FREQ bound to: 100000000 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_UART_ASYNC bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT1 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT1_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT2 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT2_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT3 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT3_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT4 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT4_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT1_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT1_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT1_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT2_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT2_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT2_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT3_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT3_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT3_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT4_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT4_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT4_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO1_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO1_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO2_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO2_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO3_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO3_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO4_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO4_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI1_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI2_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI3_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI4_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_INTC_USE_EXT_INTR bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_INTC_INTR_SIZE bound to: 1 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_INTC_LEVEL_EDGE bound to: 16'b0000000000000000 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_POSITIVE bound to: 16'b1111111111111111 
h p
x
� 
R
%s
*synth2:
8	Parameter C_INTC_HAS_FAST bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_ADDR_WIDTH bound to: 17 - type: integer 
h p
x
� 
�
%s
*synth2q
o	Parameter C_INTC_BASE_VECTORS bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
W
%s
*synth2?
=	Parameter C_INTC_ASYNC_INTR bound to: 16'b1111111111111111 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

iomodule2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
98812
U02

iomodule2x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/synth/bd_3914_iomodule_0_0.vhd2
2968@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

iomodule2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2	
100928@Z8-638h px� 
H
%s
*synth20
.	Parameter C_AW bound to: 32 - type: integer 
h p
x
� 
{
%s
*synth2c
a	Parameter C_BAR bound to: 64'b0000000000000000000000000000000010000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
pselect_mask2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97142
pselect_mask_reg2
pselect_mask2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2	
104448@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
iomodule_v3_1_10_pselect_mask2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97298@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
iomodule_v3_1_10_pselect_mask2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97298@Z8-256h px� 
H
%s
*synth20
.	Parameter C_AW bound to: 32 - type: integer 
h p
x
� 
{
%s
*synth2c
a	Parameter C_BAR bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
|
%s
*synth2d
b	Parameter C_MASK bound to: 64'b0000000000000000000000000000000011000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
pselect_mask2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97142
pselect_mask_io2
pselect_mask2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2	
104898@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2/
-iomodule_v3_1_10_pselect_mask__parameterized02v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97298@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2/
-iomodule_v3_1_10_pselect_mask__parameterized02
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
97298@Z8-256h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FREQ bound to: 100000000 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_AVOID_PRIMITIVES bound to: 0 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_RX bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_TX bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_BAUDRATE bound to: 9600 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_UART_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_USE_PARITY bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_ODD_PARITY bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_RX_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_TX_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_UART_ERROR_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_UART_PROG_BAUDRATE bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_FREQ bound to: 100000000 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_UART_ASYNC bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT1 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT1_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT2 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT2_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT3 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT3_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_FIT4 bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_FIT4_No_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FIT4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT1_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT1_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT1_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT2_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT2_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT2_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT3_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT3_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT3_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_PIT4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_PIT4_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_PIT4_READABLE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT4_PRESCALER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_PIT4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO1_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO1_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO2_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO2_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO3_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO3_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPO4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPO4_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_GPO4_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI1 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI1_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI1_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI2 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI2_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI2_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI3 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI3_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI3_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_GPI4 bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_GPI4_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_GPI4_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_INTC_USE_EXT_INTR bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_INTC_INTR_SIZE bound to: 1 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_INTC_LEVEL_EDGE bound to: 16'b0000000000000000 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_POSITIVE bound to: 16'b1111111111111111 
h p
x
� 
R
%s
*synth2:
8	Parameter C_INTC_HAS_FAST bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_ADDR_WIDTH bound to: 17 - type: integer 
h p
x
� 
�
%s
*synth2q
o	Parameter C_INTC_BASE_VECTORS bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
W
%s
*synth2?
=	Parameter C_INTC_ASYNC_INTR bound to: 16'b1111111111111111 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
Iomodule_core2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
85022
IOModule_Core_I12
iomodule_core2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2	
106808@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
Iomodule_core2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
87168@Z8-638h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FREQ bound to: 100000000 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_UART_FREQ bound to: 100000000 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_VOTE_SIZE bound to: 199 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
W
%s
*synth2?
=	Parameter C_UART_PROG_BAUDRATE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_UART_ASYNC bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_BAUDRATE bound to: 9600 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_RX bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_TX bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_UART_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_USE_PARITY bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_ODD_PARITY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
UART2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
76802	
UART_I12
UART2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
90428@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
iomodule_v3_1_10_UART2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
77318@Z8-638h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FREQ bound to: 100000000 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_VOTE_SIZE bound to: 136 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
W
%s
*synth2?
=	Parameter C_UART_PROG_BAUDRATE bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_UART_BAUDRATE bound to: 9600 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_RX bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_UART_TX bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_UART_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_USE_PARITY bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_UART_ODD_PARITY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	UART_Core2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
71342
UART_Core_I2
	UART_Core2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
78508@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
	UART_Core2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
71788@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
	UART_Core2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
71788@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
iomodule_v3_1_10_UART2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
77318@Z8-256h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 34 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_FIT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_NO_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_INACCURACY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

FIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
24932
FIT_I12

FIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
90958@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

FIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
25218@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

FIT_Module2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
25218@Z8-256h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 34 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_FIT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_NO_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_INACCURACY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

FIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
24932
FIT_I22

FIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
91188@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 34 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_FIT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_NO_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_INACCURACY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

FIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
24932
FIT_I32

FIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
91418@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 34 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_FIT bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_NO_CLOCKS bound to: 6216 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_INACCURACY bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

FIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
24932
FIT_I42

FIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
91648@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 70 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_PIT bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PIT_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_PIT_READABLE bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

PIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
42732
PIT_I12

PIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
92028@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

PIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
43048@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

PIT_Module2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
43048@Z8-256h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 70 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_PIT bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PIT_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_PIT_READABLE bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

PIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
42732
PIT_I22

PIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
92418@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 70 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_PIT bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PIT_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_PIT_READABLE bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

PIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
42732
PIT_I32

PIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
92808@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 70 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_PIT bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PIT_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_PIT_READABLE bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

PIT_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
42732
PIT_I42

PIT_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
93198@Z8-3491h px� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 32 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_GPO bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPO_SIZE bound to: 32 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_GPO_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPO_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
31862
GPO_I12

GPO_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
93478@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

GPO_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
32108@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

GPO_Module2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
32108@Z8-256h px� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 32 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_GPO bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPO_SIZE bound to: 32 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_GPO_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPO_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
31862
GPO_I22

GPO_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
93668@Z8-3491h px� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 32 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_GPO bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPO_SIZE bound to: 32 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_GPO_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPO_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
31862
GPO_I32

GPO_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
93858@Z8-3491h px� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_VOTE_SIZE bound to: 32 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_GPO bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPO_SIZE bound to: 32 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_GPO_INIT bound to: 32'b00000000000000000000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPO_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
31862
GPO_I42

GPO_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
94048@Z8-3491h px� 
L
%s
*synth24
2	Parameter C_USE_GPI bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPI_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_GPI_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPI_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30152
GPI_I12

GPI_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
94268@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

GPI_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30318@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

GPI_Module2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30318@Z8-256h px� 
L
%s
*synth24
2	Parameter C_USE_GPI bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPI_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_GPI_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPI_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30152
GPI_I22

GPI_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
94428@Z8-3491h px� 
L
%s
*synth24
2	Parameter C_USE_GPI bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPI_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_GPI_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPI_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30152
GPI_I32

GPI_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
94588@Z8-3491h px� 
L
%s
*synth24
2	Parameter C_USE_GPI bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_GPI_SIZE bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_GPI_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

GPI_Module2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
30152
GPI_I42

GPI_Module2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
94748@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
H
%s
*synth20
.	Parameter C_TMR bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_USE_TMR_DISABLE bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_VOTE_SIZE bound to: 170 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_INTC_ENABLED bound to: 0 - type: integer 
h p
x
� 
g
%s
*synth2O
M	Parameter C_INTC_LEVEL_EDGE bound to: 32'b00000000000000001111111111111111 
h p
x
� 
e
%s
*synth2M
K	Parameter C_INTC_POSITIVE bound to: 32'b11111111111111111111111111111111 
h p
x
� 
g
%s
*synth2O
M	Parameter C_INTC_ASYNC_INTR bound to: 32'b11111111111111110000000000000000 
h p
x
� 
R
%s
*synth2:
8	Parameter C_INTC_HAS_FAST bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_ADDR_WIDTH bound to: 17 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_INTC_NUM_SYNC_FF bound to: 2 - type: integer 
h p
x
� 
�
%s
*synth2q
o	Parameter C_INTC_BASE_VECTORS bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_LUTRAM bound to: yes - type: string 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	intr_ctrl2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
33652
intr_ctrl_I12
	intr_ctrl2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
95168@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
	intr_ctrl2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
34118@Z8-638h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
iomodule_v3_1_10_MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10318@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10608@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
iomodule_v3_1_10_MB_FDR2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10318@Z8-256h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
>
%s
*synth2&
$	Parameter C_TARGET bound to: 1'b0 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_FDR2t
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
10152
fdr_i2
MB_FDR2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
41708@Z8-3491h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
	intr_ctrl2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
34118@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
Iomodule_core2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2
87168@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

iomodule2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/hdl/iomodule_v3_1_vh_rfs.vhd2	
100928@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_iomodule_0_02
02
12x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/synth/bd_3914_iomodule_0_0.vhd2
858@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_lmb_bram_I_02x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_6/synth/bd_3914_lmb_bram_I_0.vhd2
808@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
V
%s
*synth2>
<	Parameter C_XDEVICEFAMILY bound to: artix7 - type: string 
h p
x
� 
T
%s
*synth2<
:	Parameter C_ELABORATION_DIR bound to: ./ - type: string 
h p
x
� 
S
%s
*synth2;
9	Parameter C_INTERFACE_TYPE bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_AXI_TYPE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_AXI_SLAVE_TYPE bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_USE_BRAM_BLOCK bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_ENABLE_32BIT_ADDRESS bound to: 1 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_CTRL_ECC_ALGO bound to: NONE - type: string 
h p
x
� 
O
%s
*synth27
5	Parameter C_HAS_AXI_ID bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_AXI_ID_WIDTH bound to: 4 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_MEM_TYPE bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_BYTE_SIZE bound to: 8 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_ALGORITHM bound to: 1 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PRIM_TYPE bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_LOAD_INIT_FILE bound to: 0 - type: integer 
h p
x
� 
c
%s
*synth2K
I	Parameter C_INIT_FILE_NAME bound to: no_coe_file_loaded - type: string 
h p
x
� 
d
%s
*synth2L
J	Parameter C_INIT_FILE bound to: bd_3914_lmb_bram_I_0.mem - type: string 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_DEFAULT_DATA bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_DEFAULT_DATA bound to: 0 - type: string 
h p
x
� 
M
%s
*synth25
3	Parameter C_HAS_RSTA bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_RST_PRIORITY_A bound to: CE - type: string 
h p
x
� 
M
%s
*synth25
3	Parameter C_RSTRAM_A bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_INITA_VAL bound to: 0 - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_HAS_ENA bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_HAS_REGCEA bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_USE_BYTE_WEA bound to: 1 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_WEA_WIDTH bound to: 4 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_WRITE_MODE_A bound to: WRITE_FIRST - type: string 
h p
x
� 
S
%s
*synth2;
9	Parameter C_WRITE_WIDTH_A bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_READ_WIDTH_A bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_WRITE_DEPTH_A bound to: 32768 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_READ_DEPTH_A bound to: 32768 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_ADDRA_WIDTH bound to: 32 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_HAS_RSTB bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_RST_PRIORITY_B bound to: CE - type: string 
h p
x
� 
M
%s
*synth25
3	Parameter C_RSTRAM_B bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_INITB_VAL bound to: 0 - type: string 
h p
x
� 
L
%s
*synth24
2	Parameter C_HAS_ENB bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_HAS_REGCEB bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_USE_BYTE_WEB bound to: 1 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_WEB_WIDTH bound to: 4 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_WRITE_MODE_B bound to: WRITE_FIRST - type: string 
h p
x
� 
S
%s
*synth2;
9	Parameter C_WRITE_WIDTH_B bound to: 32 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_READ_WIDTH_B bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_WRITE_DEPTH_B bound to: 32768 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_READ_DEPTH_B bound to: 32768 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_ADDRB_WIDTH bound to: 32 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_HAS_MEM_OUTPUT_REGS_A bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_HAS_MEM_OUTPUT_REGS_B bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_HAS_MUX_OUTPUT_REGS_A bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_HAS_MUX_OUTPUT_REGS_B bound to: 0 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_MUX_PIPELINE_STAGES bound to: 0 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_HAS_SOFTECC_INPUT_REGS_A bound to: 0 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_HAS_SOFTECC_OUTPUT_REGS_B bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_SOFTECC bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_ECC bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_EN_ECC_PIPE bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_READ_LATENCY_A bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_READ_LATENCY_B bound to: 1 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_HAS_INJECTERR bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_SIM_COLLISION_CHECK bound to: ALL - type: string 
h p
x
� 
O
%s
*synth27
5	Parameter C_COMMON_CLK bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_DISABLE_WARN_BHV_COLL bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_EN_SLEEP_PIN bound to: 0 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_URAM bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_EN_RDADDRA_CHG bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_EN_RDADDRB_CHG bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_EN_DEEPSLEEP_PIN bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_EN_SHUTDOWN_PIN bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_EN_SAFETY_CKT bound to: 1 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_DISABLE_WARN_BHV_RANGE bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_COUNT_36K_BRAM bound to: 32 - type: string 
h p
x
� 
R
%s
*synth2:
8	Parameter C_COUNT_18K_BRAM bound to: 0 - type: string 
h p
x
� 
~
%s
*synth2f
d	Parameter C_EST_POWER_SUMMARY bound to: Estimated Power for IP     :     20.388 mW - type: string 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
blk_mem_gen_v8_4_82z
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_6/hdl/blk_mem_gen_v8_4_vhsyn_rfs.vhd2
1952082
U02
blk_mem_gen_v8_4_82x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_6/synth/bd_3914_lmb_bram_I_0.vhd2
2558@Z8-3491h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_lmb_bram_I_02
02
12x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_6/synth/bd_3914_lmb_bram_I_0.vhd2
808@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	rsta_busy2
bd_3914_lmb_bram_I_02

lmb_bram_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
2628@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	rstb_busy2
bd_3914_lmb_bram_I_02

lmb_bram_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
2628@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2

lmb_bram_I2
bd_3914_lmb_bram_I_02
162
142a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
2628@Z8-7023h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_mdm_0_02s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/synth/bd_3914_mdm_0_0.vhd2
948@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
Q
%s
*synth29
7	Parameter C_DEVICE bound to: xc7a100t - type: string 
h p
x
� 
Q
%s
*synth29
7	Parameter C_REVISION bound to: (null) - type: string 
h p
x
� 
O
%s
*synth27
5	Parameter C_JTAG_CHAIN bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_USE_BSCAN bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_BSCANID bound to: 76547328 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DEBUG_INTERFACE bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_AVOID_PRIMITIVES bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_INTERCONNECT bound to: 2 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_MB_DBG_PORTS bound to: 1 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_UART bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_REG_ACCESS bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_MEM_ACCESS bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_USE_CROSS_TRIGGER bound to: 0 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_EXT_TRIG_RESET_VALUE bound to: 20'b11110001001000110100 
h p
x
� 
Q
%s
*synth29
7	Parameter C_TRACE_OUTPUT bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_TRACE_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_TRACE_CLK_FREQ_HZ bound to: 200000000 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_TRACE_CLK_OUT_PHASE bound to: 90 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_TRACE_ASYNC_RESET bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_TRACE_PROTOCOL bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_TRACE_ID bound to: 110 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_S_AXI_ADDR_WIDTH bound to: 4 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_S_AXI_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
_
%s
*synth2G
E	Parameter C_S_AXI_ACLK_FREQ_HZ bound to: 100000000 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_THREAD_ID_WIDTH bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_ADDR_SIZE bound to: 32 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_DATA_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_M_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_M_AXIS_ID_WIDTH bound to: 7 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MDM2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
126812
U02
MDM2s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/synth/bd_3914_mdm_0_0.vhd2
17038@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
MDM2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
143658@Z8-638h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter DISABLE_JTAG bound to: FALSE - type: string 
h p
x
� 
M
%s
*synth25
3	Parameter JTAG_CHAIN bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

MB_BSCANE22o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3272	
BSCAN_I2

MB_BSCANE22q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
159828@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

MB_BSCANE22q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3518@Z8-638h px� 
R
%s
*synth2:
8	Parameter DISABLE_JTAG bound to: FALSE - type: string 
h p
x
� 
M
%s
*synth25
3	Parameter JTAG_CHAIN bound to: 2 - type: integer 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
	BSCANE2_I2	
BSCANE22q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3618@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

MB_BSCANE22
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3518@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
;
%s
*synth2#
!	Parameter INIT bound to: 2'b10 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_LUT12o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
15402
LUT1_I2	
MB_LUT12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
160008@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_LUT12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
15548@Z8-638h px� 
;
%s
*synth2#
!	Parameter INIT bound to: 2'b10 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
LUT12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
15768@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_LUT12
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
15548@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_BUFG2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3892
	BUFG_DRCK2	
MB_BUFG2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
165218@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2	
MB_BUFG2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
4028@Z8-638h px� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
BUFG2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
4128@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2	
MB_BUFG2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
4028@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_JTAG_CHAIN bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_USE_BSCAN bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DEBUG_INTERFACE bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_MB_DBG_PORTS bound to: 1 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_EN_WIDTH bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_REG_ACCESS bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_REG_NUM_CE bound to: 4 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_REG_DATA_WIDTH bound to: 8 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_MEM_ACCESS bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_S_AXI_ADDR_WIDTH bound to: 4 - type: integer 
h p
x
� 
_
%s
*synth2G
E	Parameter C_S_AXI_ACLK_FREQ_HZ bound to: 100000000 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_USE_CROSS_TRIGGER bound to: 0 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_EXT_TRIG_RESET_VALUE bound to: 20'b11110001001000110100 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_UART bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_UART_WIDTH bound to: 8 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_TRACE_OUTPUT bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_TRACE_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_TRACE_ASYNC_RESET bound to: 0 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_TRACE_CLK_FREQ_HZ bound to: 200000000 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_TRACE_CLK_OUT_PHASE bound to: 90 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_TRACE_PROTOCOL bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_TRACE_ID bound to: 110 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_M_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_M_AXIS_ID_WIDTH bound to: 7 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

MDM_Core2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
58002
MDM_Core_I12

MDM_Core2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
166198@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

MDM_Core2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
70378@Z8-638h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
config_scan_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
77208@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2$
"mdm_v3_2_26_xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3128@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2$
"mdm_v3_2_26_xil_scan_reset_control2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3128@Z8-256h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
sel_n_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
77308@Z8-3491h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
shift_n_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
77408@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_FDRE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5722
TX_Buffer_Empty_FDRE2	
MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
79798@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5898@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
6188@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_FDRE2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5898@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_USE_BSCAN bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_MB_DBG_PORTS bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DEBUG_INTERFACE bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_REG_ACCESS bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DBG_MEM_ACCESS bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_M_AXI_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_USE_CROSS_TRIGGER bound to: 0 - type: integer 
h p
x
� 
`
%s
*synth2H
F	Parameter C_EXT_TRIG_RESET_VALUE bound to: 20'b11110001001000110100 
h p
x
� 
M
%s
*synth25
3	Parameter C_USE_UART bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_UART_WIDTH bound to: 8 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_TRACE_OUTPUT bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_TRACE_PROTOCOL bound to: 1 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter C_EN_WIDTH bound to: 1 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
JTAG_CONTROL2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
30842
JTAG_CONTROL_I2
JTAG_CONTROL2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
104318@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
JTAG_CONTROL2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
33578@Z8-638h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
config_with_scan_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
36668@Z8-3491h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
sel_with_scan_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
36748@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

MB_FDC_12o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
7532
FDC_I2

MB_FDC_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
37318@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2

MB_FDC_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
7698@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
FDC_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
7908@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

MB_FDC_12
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
7698@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_FDRE_12o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8122
	SYNC_FDRE2
	MB_FDRE_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
37418@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
	MB_FDRE_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8298@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
FDRE_12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8528@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
	MB_FDRE_12
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8298@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_STATIC bound to: 1 - type: bool 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000101100111 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_12
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
39358@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-638h px� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000101100111 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
MB_SRL16E_I12
SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9038@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_SRL16E2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_STATIC bound to: 1 - type: bool 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0100001010100111 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_22
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
39538@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized02q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-638h px� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0100001010100111 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
MB_SRL16E_I12
SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9038@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized02
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_STATIC bound to: 1 - type: bool 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0100010001000011 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752
SRL16E_ID_12
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
40248@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-638h px� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0100010001000011 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
MB_SRL16E_I12
SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9038@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized12
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_STATIC bound to: 1 - type: bool 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0101100001001101 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752
SRL16E_ID_22
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
40428@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized22q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-638h px� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0101100001001101 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
MB_SRL16E_I12
SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9038@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized22
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

MB_FDRSE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9602
Ext_BRK_FDRSE2

MB_FDRSE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
44678@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_FDRSE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9868@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
D
%s
*synth2,
*	Parameter IS_CE_INVERTED bound to: 1'b0 
h p
x
� 
C
%s
*synth2+
)	Parameter IS_C_INVERTED bound to: 1'b0 
h p
x
� 
C
%s
*synth2+
)	Parameter IS_D_INVERTED bound to: 1'b0 
h p
x
� 
C
%s
*synth2+
)	Parameter IS_R_INVERTED bound to: 1'b0 
h p
x
� 
C
%s
*synth2+
)	Parameter IS_S_INVERTED bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
FDRSE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10428@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_FDRSE2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9868@Z8-256h px� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
xil_scan_reset_control2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
3032
data_cmd_reset_i2
xil_scan_reset_control2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
45108@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
K
%s
*synth23
1	Parameter C_DEPTH bound to: 16 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

SRL_FIFO2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
19592
	RX_FIFO_I2

SRL_FIFO2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
46118@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_SRL_FIFO2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
19798@Z8-638h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_MUXCY_XORCY2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10712
	MUXCY_L_I2
MB_MUXCY_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21098@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_MUXCY_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10878@Z8-638h px� 
�
,binding component instance '%s' to cell '%s'113*oasys2
	Native_I12	
MUXCY_L2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10988@Z8-113h px� 
�
,binding component instance '%s' to cell '%s'113*oasys2
	Native_I22
XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
11058@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_MUXCY_XORCY2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10878@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_FDRE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5722
FDRE_I2	
MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21338@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_MUXCY_XORCY2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10712
	MUXCY_L_I2
MB_MUXCY_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21098@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_FDRE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5722
FDRE_I2	
MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21338@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
MB_MUXCY_XORCY2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
10712
	MUXCY_L_I2
MB_MUXCY_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21098@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_FDRE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5722
FDRE_I2	
MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21338@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2

MB_XORCY2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
11672	
XORCY_I2

MB_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21238@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2
mdm_v3_2_26_MB_XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
11818@Z8-638h px� 
�
,binding component instance '%s' to cell '%s'113*oasys2
Native2
XORCY2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
11918@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_MB_XORCY2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
11818@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2	
MB_FDRE2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
5722
FDRE_I2	
MB_FDRE2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21338@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_I2
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21508@Z8-3491h px� 
�
synthesizing module '%s'638*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized32q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-638h px� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
MB_SRL16E_I12
SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
9038@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2'
%mdm_v3_2_26_MB_SRL16E__parameterized32
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8998@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_I2
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21508@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_I2
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21508@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_I2
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21508@Z8-3491h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
Hmodule '%s' declared at '%s:%s' bound to instance '%s' of component '%s'3392*oasys2
	MB_SRL16E2o
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
8752

SRL16E_I2
	MB_SRL16E2q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
21508@Z8-3491h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-34912
100Z17-14h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
J
%s
*synth22
0	Parameter INIT bound to: 16'b0000000000000000 
h p
x
� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
mdm_v3_2_26_SRL_FIFO2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
19798@Z8-256h px� 
K
%s
*synth23
1	Parameter C_TARGET bound to: 2 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_DATA_BITS bound to: 8 - type: integer 
h p
x
� 
K
%s
*synth23
1	Parameter C_DEPTH bound to: 16 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_SRL16 bound to: yes - type: string 
h p
x
� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
JTAG_CONTROL2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
33578@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

MDM_Core2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2
70378@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
axi_lite_ipif2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
29488@Z8-638h px� 
�
synthesizing module '%s'638*oasys2
slave_attachment2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
23418@Z8-638h px� 
�
synthesizing module '%s'638*oasys2
address_decoder2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
17758@Z8-638h px� 
�
synthesizing module '%s'638*oasys2
	pselect_f2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
	pselect_f2
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
pselect_f__parameterized02{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
pselect_f__parameterized02
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
pselect_f__parameterized12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
pselect_f__parameterized12
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
pselect_f__parameterized22{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
pselect_f__parameterized22
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
15348@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
address_decoder2
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
17758@Z8-256h px� 
�
default block is never used226*oasys2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
25508@Z8-226h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
slave_attachment2
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
23418@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
axi_lite_ipif2
02
12{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
29488@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
MDM2
02
12q
m/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/mdm_v3_2_vh_rfs.vhd2	
143658@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_mdm_0_02
02
12s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/synth/bd_3914_mdm_0_0.vhd2
948@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	Interrupt2
bd_3914_mdm_0_02
mdm_02a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
2778@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
mdm_02
bd_3914_mdm_0_02
302
292a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
2778@Z8-7023h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_microblaze_I_02z
v/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_0/synth/bd_3914_microblaze_I_0.vhd2
1308@Z8-638h px� 
H
%s
*synth20
.	Parameter C_SCO bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_FREQ bound to: 100000000 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_USE_CONFIG_RESET bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_NUM_SYNC_FF_CLK bound to: 2 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_NUM_SYNC_FF_CLK_IRQ bound to: 1 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_NUM_SYNC_FF_CLK_DEBUG bound to: 2 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_NUM_SYNC_FF_DBG_CLK bound to: 1 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_NUM_SYNC_FF_DBG_TRACE_CLK bound to: 2 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_FAULT_TOLERANT bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_ECC_USE_CE_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_LOCKSTEP_SLAVE bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_LOCKSTEP_MASTER bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_TEMPORAL_DEPTH bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_ENDIANNESS bound to: 1 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
W
%s
*synth2?
=	Parameter C_PART bound to: xc7a100tcsg324-1 - type: string 
h p
x
� 
O
%s
*synth27
5	Parameter C_DATA_SIZE bound to: 32 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_LMB_DATA_SIZE bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_INSTR_SIZE bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_IADDR_SIZE bound to: 32 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_PIADDR_SIZE bound to: 32 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_DADDR_SIZE bound to: 32 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_INSTANCE bound to: bd_3914_microblaze_I_0 - type: string 
h p
x
� 
U
%s
*synth2=
;	Parameter C_AVOID_PRIMITIVES bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_AREA_OPTIMIZED bound to: 1 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_OPTIMIZATION bound to: 0 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_INTERCONNECT bound to: 2 - type: integer 
h p
x
� 
�
%s
*synth2l
j	Parameter C_BASE_VECTORS bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
]
%s
*synth2E
C	Parameter C_M_AXI_DP_THREAD_ID_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DP_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DP_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_M_AXI_DP_EXCLUSIVE_ACCESS bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_D_BUS_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_M_AXI_IP_THREAD_ID_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IP_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IP_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_I_BUS_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
J
%s
*synth22
0	Parameter C_D_LMB bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_D_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
J
%s
*synth22
0	Parameter C_D_AXI bound to: 1 - type: integer 
h p
x
� 
J
%s
*synth22
0	Parameter C_I_LMB bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_I_LMB_PROTOCOL bound to: 0 - type: integer 
h p
x
� 
J
%s
*synth22
0	Parameter C_I_AXI bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter G_TEMPLATE_LIST bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_USE_MSR_INSTR bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_USE_PCMP_INSTR bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_BARREL bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_DIV bound to: 0 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_HW_MUL bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_FPU bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_USE_REORDER_INSTR bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_UNALIGNED_EXCEPTIONS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_ILL_OPCODE_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_DIV_ZERO_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_FPU_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_FSL_LINKS bound to: 0 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_USE_EXTENDED_FSL_INSTR bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_FSL_EXCEPTION bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_USE_STACK_PROTECTION bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_IMPRECISE_EXCEPTIONS bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_USE_INTERRUPT bound to: 0 - type: integer 
h p
x
� 
P
%s
*synth28
6	Parameter C_USE_EXT_BRK bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_USE_EXT_NM_BRK bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_USE_NON_SECURE bound to: 0 - type: integer 
h p
x
� 
L
%s
*synth24
2	Parameter C_USE_MMU bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_MMU_DTLB_SIZE bound to: 4 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_MMU_ITLB_SIZE bound to: 2 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_MMU_TLB_ACCESS bound to: 3 - type: integer 
h p
x
� 
O
%s
*synth27
5	Parameter C_MMU_ZONES bound to: 16 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_MMU_PRIVILEGED_INSTR bound to: 0 - type: integer 
h p
x
� 
\
%s
*synth2D
B	Parameter C_USE_BRANCH_TARGET_CACHE bound to: 0 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_BRANCH_TARGET_CACHE_SIZE bound to: 0 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter C_PC_WIDTH bound to: 17 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_PVR bound to: 0 - type: integer 
h p
x
� 
H
%s
*synth20
.	Parameter C_PVR_USER1 bound to: 8'b00000000 
h p
x
� 
a
%s
*synth2I
G	Parameter C_PVR_USER2 bound to: 32'b00000000000000000000000000000000 
h p
x
� 
W
%s
*synth2?
=	Parameter C_DYNAMIC_BUS_SIZING bound to: 0 - type: integer 
h p
x
� 
a
%s
*synth2I
G	Parameter C_RESET_MSR bound to: 32'b00000000000000000000000000000000 
h p
x
� 
W
%s
*synth2?
=	Parameter C_OPCODE_0x0_ILLEGAL bound to: 0 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_DEBUG_ENABLED bound to: 1 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DEBUG_INTERFACE bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_NUMBER_OF_PC_BRK bound to: 1 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_NUMBER_OF_RD_ADDR_BRK bound to: 0 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_NUMBER_OF_WR_ADDR_BRK bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_DEBUG_EVENT_COUNTERS bound to: 5 - type: integer 
h p
x
� 
[
%s
*synth2C
A	Parameter C_DEBUG_LATENCY_COUNTERS bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_DEBUG_COUNTER_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_DEBUG_TRACE_SIZE bound to: 8192 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_DEBUG_EXTERNAL_TRACE bound to: 0 - type: integer 
h p
x
� 
\
%s
*synth2D
B	Parameter C_DEBUG_TRACE_ASYNC_RESET bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_DEBUG_PROFILE_SIZE bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_INTERRUPT_IS_EDGE bound to: 0 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_EDGE_IS_POSITIVE bound to: 1 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_ASYNC_INTERRUPT bound to: 1 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_ASYNC_WAKEUP bound to: 3 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M0_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S0_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M1_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S1_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M2_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S2_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M3_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S3_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M4_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S4_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M5_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S5_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M6_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S6_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M7_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S7_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M8_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S8_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_M9_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_S9_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M10_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S10_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M11_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S11_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M12_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S12_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M13_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S13_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M14_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S14_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M15_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_S15_AXIS_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
�
%s
*synth2o
m	Parameter C_ICACHE_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
�
%s
*synth2o
m	Parameter C_ICACHE_HIGHADDR bound to: 64'b0000000000000000000000000000000000111111111111111111111111111111 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_ICACHE bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_ALLOW_ICACHE_WR bound to: 1 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_ADDR_TAG_BITS bound to: 0 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_CACHE_BYTE_SIZE bound to: 8192 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_ICACHE_LINE_LEN bound to: 4 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_ICACHE_ALWAYS_USED bound to: 1 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_ICACHE_STREAMS bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_ICACHE_VICTIMS bound to: 0 - type: integer 
h p
x
� 
\
%s
*synth2D
B	Parameter C_ICACHE_FORCE_TAG_LUTRAM bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_ICACHE_DATA_WIDTH bound to: 0 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_M_AXI_IC_THREAD_ID_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_USER_VALUE bound to: 31 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_IC_AWUSER_WIDTH bound to: 5 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_IC_ARUSER_WIDTH bound to: 5 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_WUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_RUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_IC_BUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
�
%s
*synth2o
m	Parameter C_DCACHE_BASEADDR bound to: 64'b0000000000000000000000000000000000000000000000000000000000000000 
h p
x
� 
�
%s
*synth2o
m	Parameter C_DCACHE_HIGHADDR bound to: 64'b0000000000000000000000000000000000111111111111111111111111111111 
h p
x
� 
O
%s
*synth27
5	Parameter C_USE_DCACHE bound to: 0 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_ALLOW_DCACHE_WR bound to: 1 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DCACHE_ADDR_TAG bound to: 0 - type: integer 
h p
x
� 
X
%s
*synth2@
>	Parameter C_DCACHE_BYTE_SIZE bound to: 8192 - type: integer 
h p
x
� 
T
%s
*synth2<
:	Parameter C_DCACHE_LINE_LEN bound to: 4 - type: integer 
h p
x
� 
W
%s
*synth2?
=	Parameter C_DCACHE_ALWAYS_USED bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_DCACHE_USE_WRITEBACK bound to: 0 - type: integer 
h p
x
� 
S
%s
*synth2;
9	Parameter C_DCACHE_VICTIMS bound to: 0 - type: integer 
h p
x
� 
\
%s
*synth2D
B	Parameter C_DCACHE_FORCE_TAG_LUTRAM bound to: 0 - type: integer 
h p
x
� 
V
%s
*synth2>
<	Parameter C_DCACHE_DATA_WIDTH bound to: 0 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_M_AXI_DC_THREAD_ID_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_DATA_WIDTH bound to: 32 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
^
%s
*synth2F
D	Parameter C_M_AXI_DC_EXCLUSIVE_ACCESS bound to: 0 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_USER_VALUE bound to: 31 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_DC_AWUSER_WIDTH bound to: 5 - type: integer 
h p
x
� 
Z
%s
*synth2B
@	Parameter C_M_AXI_DC_ARUSER_WIDTH bound to: 5 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_WUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_RUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
Y
%s
*synth2A
?	Parameter C_M_AXI_DC_BUSER_WIDTH bound to: 1 - type: integer 
h p
x
� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_microblaze_I_02
02
12z
v/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_0/synth/bd_3914_microblaze_I_0.vhd2
1308@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
Interrupt_Ack2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	MB_Halted2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
Sleep2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	Hibernate2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2	
Suspend2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
Dbg_Continue2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	Pause_Ack2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
M_AXI_DP_AWPROT2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
M_AXI_DP_ARPROT2
bd_3914_microblaze_I_02
microblaze_I2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
microblaze_I2
bd_3914_microblaze_I_02
662
572a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3098@Z8-7023h px� 
�
synthesizing module '%s'638*oasys2
bd_3914_rst_0_02s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/synth/bd_3914_rst_0_0.vhd2
748@Z8-638h px� 
O
%s
*synth27
5	Parameter C_FAMILY bound to: artix7 - type: string 
h p
x
� 
R
%s
*synth2:
8	Parameter C_EXT_RST_WIDTH bound to: 4 - type: integer 
h p
x
� 
R
%s
*synth2:
8	Parameter C_AUX_RST_WIDTH bound to: 4 - type: integer 
h p
x
� 
F
%s
*synth2.
,	Parameter C_EXT_RESET_HIGH bound to: 1'b1 
h p
x
� 
F
%s
*synth2.
,	Parameter C_AUX_RESET_HIGH bound to: 1'b0 
h p
x
� 
P
%s
*synth28
6	Parameter C_NUM_BUS_RST bound to: 1 - type: integer 
h p
x
� 
Q
%s
*synth29
7	Parameter C_NUM_PERP_RST bound to: 1 - type: integer 
h p
x
� 
]
%s
*synth2E
C	Parameter C_NUM_INTERCONNECT_ARESETN bound to: 1 - type: integer 
h p
x
� 
U
%s
*synth2=
;	Parameter C_NUM_PERP_ARESETN bound to: 1 - type: integer 
h p
x
� 
�
synthesizing module '%s'638*oasys2
proc_sys_reset2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
13308@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b1 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2
	FDRE_inst2
FDRE2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
13998@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b1 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2

FDRE_BSR2
FDRE2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
14158@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2

FDRE_BSR_N2
FDRE2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
14418@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b1 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2

FDRE_PER2
FDRE2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
14648@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2

FDRE_PER_N2
FDRE2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
14888@Z8-113h px� 
�
synthesizing module '%s'638*oasys2
lpf2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
8218@Z8-638h px� 
�
synthesizing module '%s'%s4497*oasys2
SRL162
 2=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2

1372468@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
SRL162
 2
02
12=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2

1372468@Z8-6155h px� 
�
synthesizing module '%s'638*oasys2

cdc_sync2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
1068@Z8-638h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2"
 CROSS_PLEVEL_IN2SCNDRY_IN_cdc_to2
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5148@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2'
%CROSS_PLEVEL_IN2SCNDRY_s_level_out_d22
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5458@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2'
%CROSS_PLEVEL_IN2SCNDRY_s_level_out_d32
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5548@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2'
%CROSS_PLEVEL_IN2SCNDRY_s_level_out_d42
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5648@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2'
%CROSS_PLEVEL_IN2SCNDRY_s_level_out_d52
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5748@Z8-113h px� 
:
%s
*synth2"
 	Parameter INIT bound to: 1'b0 
h p
x
� 
�
,binding component instance '%s' to cell '%s'113*oasys2'
%CROSS_PLEVEL_IN2SCNDRY_s_level_out_d62
FDR2r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
5848@Z8-113h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

cdc_sync2
02
12r
n/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/lib_cdc_v1_0_rfs.vhd2
1068@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
lpf2
02
12|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
8218@Z8-256h px� 
�
synthesizing module '%s'638*oasys2
sequence_psr2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
3048@Z8-638h px� 
�
synthesizing module '%s'638*oasys2	
upcnt_n2|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
1268@Z8-638h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2	
upcnt_n2
02
12|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
1268@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
sequence_psr2
02
12|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
3048@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
proc_sys_reset2
02
12|
x/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/hdl/proc_sys_reset_v5_0_vh_rfs.vhd2
13308@Z8-256h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2
bd_3914_rst_0_02
02
12s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/synth/bd_3914_rst_0_0.vhd2
748@Z8-256h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
interconnect_aresetn2
bd_3914_rst_0_02
rst_02a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3678@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
rst_02
bd_3914_rst_0_02
102
92a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
3678@Z8-7023h px� 
�
synthesizing module '%s'%s4497*oasys2
bd_3914_xlconcat_0_02
 2v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_8/synth/bd_3914_xlconcat_0_0.v2
538@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
xlconcat_v2_1_6_xlconcat2
 2t
p/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_8/hdl/xlconcat_v2_1_vl_rfs.v2
598@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
xlconcat_v2_1_6_xlconcat2
 2
02
12t
p/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_8/hdl/xlconcat_v2_1_vl_rfs.v2
598@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
bd_3914_xlconcat_0_02
 2
02
12v
r/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_8/synth/bd_3914_xlconcat_0_0.v2
538@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
bd_39142
 2
02
12a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/synth/bd_3914.v2
108@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
cpu2
 2
02
12X
T/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/synth/cpu.v2
538@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_mcs_bridge2
 2d
`/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_mcs_bridge.sv2
18@Z8-6157h px� 
U
%s
*synth2=
;	Parameter BRG_BASE bound to: -1073741824 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_mcs_bridge2
 2
02
12d
`/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_mcs_bridge.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
mmio_sys_sampler2
 2f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/mmio_sys_sampler.sv2
48@Z8-6157h px� 
H
%s
*synth20
.	Parameter N_SW bound to: 16 - type: integer 
h p
x
� 
I
%s
*synth21
/	Parameter N_LED bound to: 16 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
chu_mmio_controller2
 2i
e/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_mmio_controller.sv2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_mmio_controller2
 2
02
12i
e/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_mmio_controller.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
	chu_timer2
 2_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_timer.sv2
98@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	chu_timer2
 2
02
12_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_timer.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

chu_uart2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/chu_uart.sv2
88@Z8-6157h px� 
Q
%s
*synth29
7	Parameter FIFO_DEPTH_BIT bound to: 8 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
xil_internal_svlib_uart2
 2_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart.sv2
18@Z8-6157h px� 
G
%s
*synth2/
-	Parameter DBIT bound to: 8 - type: integer 
h p
x
� 
K
%s
*synth23
1	Parameter SB_TICK bound to: 16 - type: integer 
h p
x
� 
I
%s
*synth21
/	Parameter FIFO_W bound to: 8 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2

baud_gen2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/baud_gen.sv2
48@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

baud_gen2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/baud_gen.sv2
48@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
uart_rx2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_rx.sv2
28@Z8-6157h px� 
G
%s
*synth2/
-	Parameter DBIT bound to: 8 - type: integer 
h p
x
� 
K
%s
*synth23
1	Parameter SB_TICK bound to: 16 - type: integer 
h p
x
� 
�
-case statement is not full and has no default155*oasys2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_rx.sv2
478@Z8-155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
uart_rx2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_rx.sv2
28@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
uart_tx2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_tx.sv2
28@Z8-6157h px� 
G
%s
*synth2/
-	Parameter DBIT bound to: 8 - type: integer 
h p
x
� 
K
%s
*synth23
1	Parameter SB_TICK bound to: 16 - type: integer 
h p
x
� 
�
-case statement is not full and has no default155*oasys2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_tx.sv2
528@Z8-155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
uart_tx2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart_tx.sv2
28@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
fifo2
 2_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/fifo.sv2
28@Z8-6157h px� 
M
%s
*synth25
3	Parameter DATA_WIDTH bound to: 8 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter ADDR_WIDTH bound to: 8 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
	fifo_ctrl2
 2d
`/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/fifo_ctrl.sv2
28@Z8-6157h px� 
M
%s
*synth25
3	Parameter ADDR_WIDTH bound to: 8 - type: integer 
h p
x
� 
�
Gfound qualifier unique on case statement: implementing as parallel_case294*oasys2d
`/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/fifo_ctrl.sv2
498@Z8-294h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	fifo_ctrl2
 2
02
12d
`/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/fifo_ctrl.sv2
28@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

reg_file2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/reg_file.sv2
28@Z8-6157h px� 
M
%s
*synth25
3	Parameter DATA_WIDTH bound to: 8 - type: integer 
h p
x
� 
M
%s
*synth25
3	Parameter ADDR_WIDTH bound to: 8 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

reg_file2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/reg_file.sv2
28@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
fifo2
 2
02
12_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/fifo/fifo.sv2
28@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
xil_internal_svlib_uart2
 2
02
12_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/uart.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

chu_uart2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/uart/chu_uart.sv2
88@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
chu_gpo2
 2]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_gpo.sv2
18@Z8-6157h px� 
E
%s
*synth2-
+	Parameter W bound to: 16 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
chu_gpo2
 2
02
12]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_gpo.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
chu_gpi2
 2]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_gpi.sv2
18@Z8-6157h px� 
E
%s
*synth2-
+	Parameter W bound to: 16 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
chu_gpi2
 2
02
12]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_gpi.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_xadc_core2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_xadc_core.sv2
88@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
	xadc_fpro2
 2_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/xadc_fpro.sv2
498@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
XADC2
 2=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2

1430278@Z8-6157h px� 
M
%s
*synth25
3	Parameter INIT_40 bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_41 bound to: 16'b0010000110101111 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_42 bound to: 16'b0000010000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_48 bound to: 16'b0000001100000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_49 bound to: 16'b0000110000001100 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4A bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4B bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4C bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4D bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4E bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_4F bound to: 16'b0000000000000000 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_50 bound to: 16'b1011010111101101 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_51 bound to: 16'b0101011111100100 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_52 bound to: 16'b1010000101000111 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_53 bound to: 16'b1100101000110011 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_54 bound to: 16'b1010100100111010 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_55 bound to: 16'b0101001011000110 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_56 bound to: 16'b1001010101010101 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_57 bound to: 16'b1010111001001110 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_58 bound to: 16'b0101100110011001 
h p
x
� 
M
%s
*synth25
3	Parameter INIT_5C bound to: 16'b0101000100010001 
h p
x
� 
R
%s
*synth2:
8	Parameter SIM_DEVICE bound to: 7SERIES - type: string 
h p
x
� 
[
%s
*synth2C
A	Parameter SIM_MONITOR_FILE bound to: design.txt - type: string 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
XADC2
 2
02
12=
9/tools/Xilinx/Vivado/2024.1/scripts/rt/data/unisim_comp.v2

1430278@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	xadc_fpro2
 2
02
12_
[/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/xadc_fpro.sv2
498@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_xadc_core2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_xadc_core.sv2
88@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_io_pwm_core2
 2e
a/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_io_pwm_core.sv2
158@Z8-6157h px� 
D
%s
*synth2,
*	Parameter W bound to: 8 - type: integer 
h p
x
� 
E
%s
*synth2-
+	Parameter R bound to: 10 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_io_pwm_core2
 2
02
12e
a/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_io_pwm_core.sv2
158@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_debounce_core2
 2g
c/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_debounce_core.sv2
18@Z8-6157h px� 
D
%s
*synth2,
*	Parameter W bound to: 5 - type: integer 
h p
x
� 
E
%s
*synth2-
+	Parameter N bound to: 20 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
debounce_fsm2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/debounce_fsm.sv2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
debounce_fsm2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/debounce_fsm.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
debounce_counter2
 2f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/debounce_counter.sv2
18@Z8-6157h px� 
E
%s
*synth2-
+	Parameter N bound to: 20 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
debounce_counter2
 2
02
12f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/debounce_counter.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_debounce_core2
 2
02
12g
c/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_debounce_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_led_mux_core2
 2f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_led_mux_core.sv2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2

led_mux82
 2^
Z/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/led_mux8.sv2
18@Z8-6157h px� 
�
Gfound qualifier unique on case statement: implementing as parallel_case294*oasys2^
Z/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/led_mux8.sv2
318@Z8-294h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

led_mux82
 2
02
12^
Z/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/led_mux8.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_led_mux_core2
 2
02
12f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_led_mux_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_spi_core2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_spi_core.sv2
18@Z8-6157h px� 
D
%s
*synth2,
*	Parameter S bound to: 1 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
spi2
 2Y
U/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/spi.sv2
338@Z8-6157h px� 
�
-case statement is not full and has no default155*oasys2Y
U/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/spi.sv2
888@Z8-155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
spi2
 2
02
12Y
U/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/spi.sv2
338@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_spi_core2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_spi_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_i2c_core2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_i2c_core.sv2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2

i2c_master2
 2`
\/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/i2c_master.sv2
308@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

i2c_master2
 2
02
12`
\/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/i2c_master.sv2
308@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_i2c_core2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_i2c_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_ps2_core2
 2b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_ps2_core.sv2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter W_SIZE bound to: 8 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2	
ps2_top2
 2]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2_top.sv2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter W_SIZE bound to: 8 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
ps2tx2
 2[
W/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2tx.sv2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ps2tx2
 2
02
12[
W/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2tx.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ps2rx2
 2[
W/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2rx.sv2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ps2rx2
 2
02
12[
W/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2rx.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
ps2_top2
 2
02
12]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ps2_top.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_ps2_core2
 2
02
12b
^/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_ps2_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_ddfs_core2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_ddfs_core.sv2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
ddfs2
 2Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ddfs.sv2
18@Z8-6157h px� 
F
%s
*synth2.
,	Parameter PW bound to: 30 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2	
sin_rom2
 2]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/sin_rom.sv2
18@Z8-6157h px� 
�
%s, ignoring3604*oasys2z
xcould not open $readmem data file 'sin_table.txt'; please make sure the file is added to project and has read permission2]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/sin_rom.sv2
188@Z8-4445h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
sin_rom2
 2
02
12]
Y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/sin_rom.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ddfs2
 2
02
12Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ddfs.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ds_1bit_dac2
 2a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ds_1bit_dac.sv2
18@Z8-6157h px� 
E
%s
*synth2-
+	Parameter W bound to: 16 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ds_1bit_dac2
 2
02
12a
]/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/ds_1bit_dac.sv2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_ddfs_core2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_ddfs_core.sv2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
chu_adsr_core2
 2c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_adsr_core.sv2
108@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
adsr2
 2Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/adsr.sv2
198@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
adsr2
 2
02
12Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/adsr.sv2
198@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
chu_adsr_core2
 2
02
12c
_/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/chu_adsr_core.sv2
108@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
mmio_sys_sampler2
 2
02
12f
b/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/mmio_sys_sampler.sv2
48@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
mcs_top_sampler2
 2
02
12e
a/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/sources_1/imports/HDL/mcs_top_sampler.sv2
18@Z8-6155h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
is_read_reg2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
24478@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
is_write_reg2{
w/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd2
24488@Z8-6014h px� 
q
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2

audio_on2
1Z8-3917h px� 
s
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2

ja_btm[10]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[9]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[8]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[7]2
0Z8-3917h px� 
t
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_adsr_coreZ8-7129h px� 
w
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_adsr_coreZ8-7129h px� 
w
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_adsr_coreZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[7]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[6]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[5]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[4]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[3]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[2]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[1]2	
sin_romZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
	addr_r[0]2	
sin_romZ8-7129h px� 
t
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_ddfs_coreZ8-7129h px� 
w
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_ddfs_coreZ8-7129h px� 
w
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_ddfs_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[31]2
chu_ddfs_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[30]2
chu_ddfs_coreZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_ps2_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_ps2_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_ps2_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[2]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[31]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[30]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[29]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[28]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[27]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[26]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[25]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[24]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[23]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[22]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[21]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[20]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[19]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[18]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[17]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[16]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[15]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[14]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[13]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[12]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[11]2
chu_ps2_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[10]2
chu_ps2_coreZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2

wr_data[9]2
chu_ps2_coreZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2

wr_data[8]2
chu_ps2_coreZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_i2c_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_i2c_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_i2c_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[2]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[31]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[30]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[29]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[28]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[27]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[26]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[25]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[24]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[23]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[22]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[21]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[20]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[19]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[18]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[17]2
chu_i2c_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[16]2
chu_i2c_coreZ8-7129h px� 
s
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_spi_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_spi_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_spi_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[2]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[31]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[30]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[29]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[28]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[27]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[26]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[25]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[24]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[23]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[22]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[21]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[20]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[19]2
chu_spi_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[18]2
chu_spi_coreZ8-7129h px� 
w
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_led_mux_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_led_mux_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_led_mux_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[2]2
chu_led_mux_coreZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[1]2
chu_led_mux_coreZ8-7129h px� 
v
9Port %s in module %s is either unconnected or has no load4866*oasys2
cs2
chu_debounce_coreZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
read2
chu_debounce_coreZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
write2
chu_debounce_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[4]2
chu_debounce_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[3]2
chu_debounce_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[2]2
chu_debounce_coreZ8-7129h px� 
{
9Port %s in module %s is either unconnected or has no load4866*oasys2	
addr[1]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[31]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[30]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[29]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[28]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[27]2
chu_debounce_coreZ8-7129h px� 

9Port %s in module %s is either unconnected or has no load4866*oasys2
wr_data[26]2
chu_debounce_coreZ8-7129h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-71292
100Z17-14h px� 
�
%s*synth2�
�Finished RTL Elaboration : Time (s): cpu = 00:00:18 ; elapsed = 00:00:21 . Memory (MB): peak = 2661.062 ; gain = 823.730 ; free physical = 368 ; free virtual = 9039
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:18 ; elapsed = 00:00:21 . Memory (MB): peak = 2661.062 ; gain = 823.730 ; free physical = 368 ; free virtual = 9039
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:18 ; elapsed = 00:00:21 . Memory (MB): peak = 2661.062 ; gain = 823.730 ; free physical = 368 ; free virtual = 9039
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2
00:00:00.212

00:00:00.22

2661.0622
0.0002
3752
9046Z17-722h px� 
U
-Analyzing %s Unisim elements for replacement
17*netlist2
499Z29-17h px� 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
>

Processing XDC Constraints
244*projectZ1-262h px� 
=
Initializing timing engine
348*projectZ1-569h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2t
p/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_0/bd_3914_microblaze_I_0.xdc2!
cpu_unit/inst/microblaze_I/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2t
p/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_0/bd_3914_microblaze_I_0.xdc2!
cpu_unit/inst/microblaze_I/U0	8Z20-847h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2r
p/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_0/bd_3914_microblaze_I_0.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-236h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/bd_3914_rst_0_0_board.xdc2
cpu_unit/inst/rst_0/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2s
o/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/bd_3914_rst_0_0_board.xdc2
cpu_unit/inst/rst_0/U0	8Z20-847h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2m
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/bd_3914_rst_0_0.xdc2
cpu_unit/inst/rst_0/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2m
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/bd_3914_rst_0_0.xdc2
cpu_unit/inst/rst_0/U0	8Z20-847h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2k
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_1/bd_3914_rst_0_0.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-236h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2l
h/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_3/bd_3914_dlmb_0.xdc2
cpu_unit/inst/dlmb/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2l
h/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_3/bd_3914_dlmb_0.xdc2
cpu_unit/inst/dlmb/U0	8Z20-847h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/bd_3914_iomodule_0_0_board.xdc2
cpu_unit/inst/iomodule_0/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2x
t/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_9/bd_3914_iomodule_0_0_board.xdc2
cpu_unit/inst/iomodule_0/U0	8Z20-847h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/cpu_board.xdc2
cpu_unit/inst	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2Z
V/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/cpu_board.xdc2
cpu_unit/inst	8Z20-847h px� 
�
Parsing XDC File [%s]
179*designutils2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc8Z20-179h px� 
�
No ports matched '%s'.
584*	planAhead2
hsync2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1568@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
vsync2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1578@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[8]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1588@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[9]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1598@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
rgb[10]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1608@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
rgb[11]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1618@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[4]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1628@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[5]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1638@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[6]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1648@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[7]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1658@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[0]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1668@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[1]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1678@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[2]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1688@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
rgb[3]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1698@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2

sd_reset2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1748@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
sd_sclk2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1758@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
sd_mosi2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1768@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
sd_miso2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1778@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2	
sd_ss_n2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
1808@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_top[1]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2378@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_top[2]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2388@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_top[3]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2398@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_top[4]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2408@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_btm[7]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2418@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_btm[8]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2428@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jb_btm[9]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2438@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2

jb_btm[10]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2448@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_top[1]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2518@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_top[2]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2528@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_top[3]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2538@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_top[4]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2548@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_btm[7]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2558@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_btm[8]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2568@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jc_btm[9]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2578@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2

jc_btm[10]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2588@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_top[1]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2668@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_top[2]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2678@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_top[3]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2688@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_top[4]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2698@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_btm[7]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2708@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_btm[8]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2718@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2
	jd_btm[9]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2728@Z12-584h px�
�
No ports matched '%s'.
584*	planAhead2

jd_btm[10]2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2
2738@Z12-584h px�
�
Finished Parsing XDC File [%s]
178*designutils2}
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc8Z20-178h px� 
�
�One or more constraints failed evaluation while reading constraint file [%s] and the design contains unresolved black boxes. These constraints will be read post-synthesis (as long as their source constraint file is marked as used_in_implementation) and should be applied correctly then. You should review the constraints listed in the file [%s] and check the run log file to verify that these constraints were correctly applied.301*project2{
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-498h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2{
y/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.srcs/constrs_1/imports/M8 to M 13 - Sampler System/Nexys4_DDR_chu.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-236h px� 
�
Parsing XDC File [%s]
179*designutils2S
O/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.runs/synth_1/dont_touch.xdc8Z20-179h px� 
�
Finished Parsing XDC File [%s]
178*designutils2S
O/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.runs/synth_1/dont_touch.xdc8Z20-178h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2Q
O/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.runs/synth_1/dont_touch.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-236h px� 
�
$Parsing XDC File [%s] for cell '%s'
848*designutils2m
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/bd_3914_mdm_0_0.xdc2
cpu_unit/inst/mdm_0/U0	8Z20-848h px� 
�
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2m
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/bd_3914_mdm_0_0.xdc2
cpu_unit/inst/mdm_0/U0	8Z20-847h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2k
i/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.gen/sources_1/ip/cpu/bd_0/ip/ip_7/bd_3914_mdm_0_0.xdc2#
!.Xil/mcs_top_sampler_propImpl.xdcZ1-236h px� 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2

00:00:002

00:00:002

2725.0942
0.0002
3782
9062Z17-722h px� 
�
!Unisim Transformation Summary:
%s111*project2�
�  A total of 376 instances were transformed.
  FD => FDRE: 32 instances
  FDC_1 => FDCE (inverted pins: C): 1 instance 
  FDE => FDRE: 32 instances
  FDR => FDRE: 45 instances
  FDRE_1 => FDRE (inverted pins: C): 1 instance 
  FDRSE => FDRSE (FDRE, LUT4, VCC): 9 instances
  FDS => FDSE: 5 instances
  LUT6_2 => LUT6_2 (LUT5, LUT6): 97 instances
  MULT_AND => LUT2: 3 instances
  MUXCY_L => MUXCY: 86 instances
  RAM32X1D => RAM32X1D (RAMD32(x2)): 64 instances
  SRL16 => SRL16E: 1 instance 
Z1-111h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
 Constraint Validation Runtime : 2
00:00:00.052
00:00:00.062

2725.0942
0.0002
3772
9062Z17-722h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Constraint Validation : Time (s): cpu = 00:00:28 ; elapsed = 00:00:30 . Memory (MB): peak = 2725.094 ; gain = 887.762 ; free physical = 493 ; free virtual = 9018
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
D
%s
*synth2,
*Start Loading Part and Timing Information
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Loading part: xc7a100tcsg324-1
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:28 ; elapsed = 00:00:30 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 493 ; free virtual = 9018
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
%s
*synth20
.Start Applying 'set_property' XDC Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:28 ; elapsed = 00:00:30 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 493 ; free virtual = 9018
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
t
3inferred FSM for state register '%s' in module '%s'802*oasys2
	state_reg2
slave_attachmentZ8-802h px� 
o
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2	
uart_rxZ8-802h px� 
o
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2	
uart_txZ8-802h px� 
t
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2
debounce_fsmZ8-802h px� 
k
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2
spiZ8-802h px� 
r
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2

i2c_masterZ8-802h px� 
m
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2
ps2txZ8-802h px� 
m
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2
ps2rxZ8-802h px� 
l
3inferred FSM for state register '%s' in module '%s'802*oasys2
state_reg_reg2
adsrZ8-802h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                 sm_idle |                             0010 |                               00
h p
x
� 
y
%s
*synth2a
_                 sm_read |                             1000 |                               01
h p
x
� 
y
%s
*synth2a
_                sm_write |                             0100 |                               10
h p
x
� 
y
%s
*synth2a
_                 sm_resp |                             0001 |                               11
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
	state_reg2	
one-hot2
slave_attachmentZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                               00 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                   start |                               01 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                    data |                               10 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                    stop |                               11 | 00000000000000000000000000000011
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2

sequential2	
uart_rxZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                               00 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                   start |                               01 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                    data |                               10 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                    stop |                               11 | 00000000000000000000000000000011
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2

sequential2	
uart_txZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    zero |                              000 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                 wait1_1 |                              001 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                 wait1_2 |                              010 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                 wait1_3 |                              011 | 00000000000000000000000000000011
h p
x
� 
y
%s
*synth2a
_                     one |                              100 | 00000000000000000000000000000100
h p
x
� 
y
%s
*synth2a
_                 wait0_1 |                              101 | 00000000000000000000000000000101
h p
x
� 
y
%s
*synth2a
_                 wait0_2 |                              110 | 00000000000000000000000000000110
h p
x
� 
y
%s
*synth2a
_                 wait0_3 |                              111 | 00000000000000000000000000000111
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2

sequential2
debounce_fsmZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                               00 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_              cpha_delay |                               01 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                      p0 |                               10 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                      p1 |                               11 | 00000000000000000000000000000011
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2

sequential2
spiZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                             0000 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                  start1 |                             0001 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                  start2 |                             0010 | 00000000000000000000000000000011
h p
x
� 
y
%s
*synth2a
_                    hold |                             0011 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                 restart |                             0100 | 00000000000000000000000000001001
h p
x
� 
y
%s
*synth2a
_                   stop1 |                             0101 | 00000000000000000000000000001010
h p
x
� 
y
%s
*synth2a
_                  iSTATE |                             0110 | 00000000000000000000000000001011
h p
x
� 

%s
*synth2
*
h p
x
� 
y
%s
*synth2a
_                   data1 |                             0111 | 00000000000000000000000000000100
h p
x
� 
y
%s
*synth2a
_                   data2 |                             1000 | 00000000000000000000000000000101
h p
x
� 
y
%s
*synth2a
_                   data3 |                             1001 | 00000000000000000000000000000110
h p
x
� 
y
%s
*synth2a
_                   data4 |                             1010 | 00000000000000000000000000000111
h p
x
� 
y
%s
*synth2a
_                data_end |                             1011 | 00000000000000000000000000001000
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2

sequential2

i2c_masterZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                           000001 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                   waitr |                           000010 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                     rts |                           000100 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                   start |                           001000 | 00000000000000000000000000000011
h p
x
� 
y
%s
*synth2a
_                    data |                           010000 | 00000000000000000000000000000100
h p
x
� 
y
%s
*synth2a
_                  iSTATE |                           100000 | 00000000000000000000000000000101
h p
x
� 

%s
*synth2
*
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2	
one-hot2
ps2txZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                              001 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                     dps |                              010 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                  iSTATE |                              100 | 00000000000000000000000000000010
h p
x
� 

%s
*synth2
*
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2	
one-hot2
ps2rxZ8-3354h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                    idle |                           000001 | 00000000000000000000000000000000
h p
x
� 
y
%s
*synth2a
_                  launch |                           000010 | 00000000000000000000000000000001
h p
x
� 
y
%s
*synth2a
_                  attack |                           000100 | 00000000000000000000000000000010
h p
x
� 
y
%s
*synth2a
_                   decay |                           001000 | 00000000000000000000000000000011
h p
x
� 
y
%s
*synth2a
_                 sustain |                           010000 | 00000000000000000000000000000100
h p
x
� 
y
%s
*synth2a
_                  iSTATE |                           100000 | 00000000000000000000000000000101
h p
x
� 

%s
*synth2
*
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
state_reg_reg2	
one-hot2
adsrZ8-3354h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:31 ; elapsed = 00:00:33 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 527 ; free virtual = 9054
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
$decloning instance '%s' (%s) to '%s'223*oasys25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I12

FIT_Module25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I2Z8-223h px� 
�
$decloning instance '%s' (%s) to '%s'223*oasys25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I12

FIT_Module25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I3Z8-223h px� 
�
$decloning instance '%s' (%s) to '%s'223*oasys25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I12

FIT_Module25
3cpu_unit/inst/iomodule_0/U0/IOModule_Core_I1/FIT_I4Z8-223h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Finished RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
q
%s
*synth2Y
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
E
%s
*synth2-
+Start Cross Boundary and Area Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px� 
q
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2

audio_on2
1Z8-3917h px� 
s
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2

ja_btm[10]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[9]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[8]2
0Z8-3917h px� 
r
+design %s has port %s driven by constant %s3447*oasys2
mcs_top_sampler2
	ja_btm[7]2
0Z8-3917h px� 
~
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2

POR_FF_I2	
lmb_v10Z8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
U0/POR_FF_I2
bd_3914_ilmb_0Z8-3332h px�
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[0].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[1].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[2].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[3].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[4].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[5].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[6].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[7].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[8].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2J
HIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[9].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[10].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[11].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[12].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[13].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[14].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[15].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[16].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[17].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[18].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[19].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[20].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[21].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[22].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[23].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[24].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[25].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[26].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[27].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[28].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[29].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[30].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2K
IIOModule_Core_I1/intr_ctrl_I1/cipr_rd_dff_all[31].fdr_i/Using_FPGA.Native2

iomoduleZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2=
;MDM_Core_I1/Use_Uart.TX_Buffer_Empty_FDRE/Using_FPGA.Native2
MDMZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2V
TMicroBlaze_Core_I/Area.Core/Decode_I/Using_FPGA.ALU_unmask_EA_FDRE/Using_FPGA.Native2

MicroBlazeZ8-3332h px�
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2T
RMicroBlaze_Core_I/Area.Core/Decode_I/Using_FPGA.ALU_long_Op_FDRE/Using_FPGA.Native2

MicroBlazeZ8-3332h px�
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2X
VMicroBlaze_Core_I/Area.Core/Decode_I/Using_FPGA.ALU_long_ALU_Op_FDRE/Using_FPGA.Native2

MicroBlazeZ8-3332h px�
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2V
TMicroBlaze_Core_I/Area.Core/Decode_I/Using_FPGA.Reg_Test_Long_FDRE/Using_FPGA.Native2

MicroBlazeZ8-3332h px�
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2l
jEXT_LPF/ACTIVE_HIGH_EXT.ACT_HI_EXT/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d52
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2l
jEXT_LPF/ACTIVE_HIGH_EXT.ACT_HI_EXT/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d62
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2f
dEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_IN_cdc_to2
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2k
iEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d22
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2k
iEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d32
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2k
iEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d42
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2k
iEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d52
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2k
iEXT_LPF/ACTIVE_LOW_AUX.ACT_LO_AUX/GENERATE_LEVEL_P_S_CDC.SINGLE_BIT.CROSS_PLEVEL_IN2SCNDRY_s_level_out_d62
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2&
$ACTIVE_LOW_BSR_OUT_DFF[0].FDRE_BSR_N2
proc_sys_resetZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys22
0ps2_unit/ps2_tx_unit/FSM_onehot_state_reg_reg[5]2
chu_ps2_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys22
0ps2_unit/ps2_rx_unit/FSM_onehot_state_reg_reg[2]2
chu_ps2_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__152
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__162
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__172
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__182
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__192
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__202
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__212
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__222
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__232
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__242
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__252
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__262
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__272
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2
ddfs_unit/pcm_reg_reg__282
chu_ddfs_coreZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2'
%adsr_unit/FSM_onehot_state_reg_reg[5]2
chu_adsr_coreZ8-3332h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:39 ; elapsed = 00:00:42 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 503 ; free virtual = 9049
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
\
%s
*synth2D
B Sort Area is  ddfs_unit/modu_0 : 0 0 : 1953 1953 : Used 1 time 0
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
@
%s
*synth2(
&Start Applying XDC Timing Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
2Moved timing constraint from pin '%s' to pin '%s'
4028*oasys2%
#cpu_unit/inst/microblaze_I/U0/Reset2$
"cpu_unit/inst/rst_0/U0/FDRE_inst/QZ8-5578h px� 
z
SMoved %s constraints on hierarchical pins to their respective driving/loading pins
4235*oasys2
1Z8-5819h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:46 ; elapsed = 00:00:49 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 497 ; free virtual = 9050
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
4
%s
*synth2
Start Timing Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Timing Optimization : Time (s): cpu = 00:00:47 ; elapsed = 00:00:51 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 503 ; free virtual = 9055
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
3
%s
*synth2
Start Technology Mapping
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Technology Mapping : Time (s): cpu = 00:00:49 ; elapsed = 00:00:52 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 510 ; free virtual = 9062
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
-
%s
*synth2
Start IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
?
%s
*synth2'
%Start Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
B
%s
*synth2*
(Finished Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished IO Insertion : Time (s): cpu = 00:00:52 ; elapsed = 00:00:55 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Start Renaming Generated Instances
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:00:52 ; elapsed = 00:00:55 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start Rebuilding User Hierarchy
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:52 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Renaming Generated Ports
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Ports : Time (s): cpu = 00:00:52 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:52 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Start Renaming Generated Nets
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Nets : Time (s): cpu = 00:00:52 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Writing Synthesis Report
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
W
%s
*synth2?
=
DSP Final Report (the ' indicates corresponding REG is set)
h p
x
� 
�
%s
*synth2�
~+--------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h p
x
� 
�
%s
*synth2�
|Module Name   | DSP Mapping | A Size | B Size | C Size | D Size | P Size | AREG | BREG | CREG | DREG | ADREG | MREG | PREG | 
h p
x
� 
�
%s
*synth2�
~+--------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h p
x
� 
�
%s
*synth2�
|chu_ddfs_core | (A*B)'      | 30     | 0      | -      | -      | 32     | 0    | 0    | -    | -    | -     | 0    | 1    | 
h p
x
� 
�
%s
*synth2�
+--------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+

h p
x
� 
/
%s
*synth2

Report BlackBoxes: 
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
| |BlackBox name |Instances |
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
/
%s*synth2

Report Cell Usage: 
h px� 
4
%s*synth2
+------+---------+------+
h px� 
4
%s*synth2
|      |Cell     |Count |
h px� 
4
%s*synth2
+------+---------+------+
h px� 
4
%s*synth2
|1     |BSCANE2  |     1|
h px� 
4
%s*synth2
|2     |BUFG     |     3|
h px� 
4
%s*synth2
|3     |CARRY4   |   124|
h px� 
4
%s*synth2
|4     |DSP48E1  |     1|
h px� 
4
%s*synth2
|5     |LUT1     |    51|
h px� 
4
%s*synth2
|6     |LUT2     |   376|
h px� 
4
%s*synth2
|7     |LUT3     |   222|
h px� 
4
%s*synth2
|8     |LUT4     |   559|
h px� 
4
%s*synth2
|9     |LUT5     |   287|
h px� 
4
%s*synth2
|10    |LUT6     |   633|
h px� 
4
%s*synth2
|12    |MULT_AND |     2|
h px� 
4
%s*synth2
|13    |MUXCY_L  |    84|
h px� 
4
%s*synth2
|14    |MUXF7    |    40|
h px� 
4
%s*synth2
|15    |RAM32X1D |    64|
h px� 
4
%s*synth2
|16    |RAM64M   |    36|
h px� 
4
%s*synth2
|17    |RAMB36E1 |    32|
h px� 
4
%s*synth2
|18    |SRL16    |     1|
h px� 
4
%s*synth2
|19    |SRL16E   |    86|
h px� 
4
%s*synth2
|20    |SRLC16E  |     8|
h px� 
4
%s*synth2
|21    |XADC     |     1|
h px� 
4
%s*synth2
|22    |XORCY    |    58|
h px� 
4
%s*synth2
|23    |FD       |    32|
h px� 
4
%s*synth2
|24    |FDCE     |  1050|
h px� 
4
%s*synth2
|25    |FDC      |     1|
h px� 
4
%s*synth2
|26    |FDE      |    32|
h px� 
4
%s*synth2
|27    |FDPE     |    19|
h px� 
4
%s*synth2
|28    |FDR      |     5|
h px� 
4
%s*synth2
|29    |FDRE     |   812|
h px� 
4
%s*synth2
|31    |FDS      |     3|
h px� 
4
%s*synth2
|32    |FDSE     |    25|
h px� 
4
%s*synth2
|33    |IBUF     |    33|
h px� 
4
%s*synth2
|34    |IOBUF    |     3|
h px� 
4
%s*synth2
|35    |OBUF     |    52|
h px� 
4
%s*synth2
|36    |OBUFT    |     1|
h px� 
4
%s*synth2
+------+---------+------+
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:00:52 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.098 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
d
%s
*synth2L
JSynthesis finished with 0 errors, 0 critical warnings and 17321 warnings.
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Runtime : Time (s): cpu = 00:00:48 ; elapsed = 00:00:51 . Memory (MB): peak = 2733.098 ; gain = 831.734 ; free physical = 508 ; free virtual = 9060
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Complete : Time (s): cpu = 00:00:53 ; elapsed = 00:00:56 . Memory (MB): peak = 2733.105 ; gain = 895.766 ; free physical = 508 ; free virtual = 9060
h p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2
00:00:00.092
00:00:00.082

2733.1052
0.0002
8542
9406Z17-722h px� 
U
-Analyzing %s Unisim elements for replacement
17*netlist2
615Z29-17h px� 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
Q
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02
0Z31-138h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2

00:00:002
00:00:00.012

2789.1252
0.0002
8482
9390Z17-722h px� 
�
!Unisim Transformation Summary:
%s111*project2�
�  A total of 300 instances were transformed.
  (MUXCY,XORCY) => CARRY4: 25 instances
  FD => FDRE: 32 instances
  FDC_1 => FDCE (inverted pins: C): 1 instance 
  FDE => FDRE: 32 instances
  FDR => FDRE: 5 instances
  FDRE_1 => FDRE (inverted pins: C): 1 instance 
  FDS => FDSE: 3 instances
  IOBUF => IOBUF (IBUF, OBUFT): 3 instances
  LUT6_2 => LUT6_2 (LUT5, LUT6): 95 instances
  MULT_AND => LUT2: 2 instances
  RAM32X1D => RAM32X1D (RAMD32(x2)): 64 instances
  RAM64M => RAM64M (RAMD64E(x4)): 36 instances
  SRL16 => SRL16E: 1 instance 
Z1-111h px� 
V
%Synth Design complete | Checksum: %s
562*	vivadotcl2

afe71944Z4-1430h px� 
C
Releasing license: %s
83*common2
	SynthesisZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
3962
2382
12
0Z4-41h px� 
L
%s completed successfully
29*	vivadotcl2
synth_designZ4-42h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
synth_design: 2

00:01:012

00:01:022

2789.1252

1303.7382
8502
9391Z17-722h px� 
�
%s peak %s Memory [%s] %s12246*common2
synth_design2

Physical2
PSS2=
;(MB): overall = 2370.029; main = 1980.466; forked = 440.023Z17-2834h px� 
�
%s peak %s Memory [%s] %s12246*common2
synth_design2	
Virtual2
VSS2>
<(MB): overall = 3828.730; main = 2789.129; forked = 1095.629Z17-2834h px� 
c
%s6*runtcl2G
ESynthesis results are not added to the cache due to CRITICAL_WARNING
h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Write ShapeDB Complete: 2
00:00:00.022

00:00:002

2813.1372
0.0002
8502
9391Z17-722h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2V
T/home/noah/Projects/ECE-4310_Lab-10/ECE-4310_Lab-10.runs/synth_1/mcs_top_sampler.dcpZ17-1381h px� 
�
Executing command : %s
56330*	planAhead2i
greport_utilization -file mcs_top_sampler_utilization_synth.rpt -pb mcs_top_sampler_utilization_synth.pbZ12-24828h px� 
\
Exiting %s at %s...
206*common2
Vivado2
Wed Nov 20 09:19:54 2024Z17-206h px� 


End Record