create_pblock pblock_pr_0
resize_pblock pblock_pr_0 -add {SLICE_X28Y100:SLICE_X55Y149 DSP48_X2Y40:DSP48_X2Y59 RAMB18_X2Y40:RAMB18_X3Y59 RAMB36_X2Y20:RAMB36_X3Y29}
resize_pblock pblock_pr_0 -add {DSP48_X2Y20:DSP48_X2Y39}
add_cells_to_pblock pblock_pr_0 [get_cells -quiet [list prio_linux_i/pr_0]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_0]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_0]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_0]
