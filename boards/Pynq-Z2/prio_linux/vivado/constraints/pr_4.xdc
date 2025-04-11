create_pblock pblock_pr_4
resize_pblock pblock_pr_4 -add {SLICE_X94Y100:SLICE_X113Y149}
resize_pblock pblock_pr_4 -add {DSP48_X4Y40:DSP48_X4Y59 RAMB18_X5Y40:RAMB18_X5Y59 RAMB36_X5Y20:RAMB36_X5Y29}
resize_pblock pblock_pr_4 -add {SLICE_X94Y50:SLICE_X103Y99}
resize_pblock pblock_pr_4 -add {DSP48_X4Y20:DSP48_X4Y39}
resize_pblock pblock_pr_4 -add {RAMB18_X4Y20:RAMB18_X4Y39 RAMB36_X4Y10:RAMB36_X4Y19}
add_cells_to_pblock pblock_pr_4 [get_cells -quiet [list prio_linux_i/pr_4]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_4]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_4]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_4]
