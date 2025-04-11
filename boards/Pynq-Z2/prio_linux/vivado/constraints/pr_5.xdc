create_pblock pblock_pr_5
resize_pblock pblock_pr_5 -add {SLICE_X104Y50:SLICE_X113Y99 RAMB18_X5Y20:RAMB18_X5Y39 RAMB36_X5Y10:RAMB36_X5Y19}
resize_pblock pblock_pr_5 -add {SLICE_X94Y0:SLICE_X113Y49 DSP48_X4Y0:DSP48_X4Y19 RAMB18_X5Y0:RAMB18_X5Y19 RAMB36_X5Y0:RAMB36_X5Y9}
resize_pblock pblock_pr_5 -add {RAMB18_X4Y0:RAMB18_X4Y19 RAMB36_X4Y0:RAMB36_X4Y9}
add_cells_to_pblock pblock_pr_5 [get_cells -quiet [list prio_linux_i/pr_5]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_5]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_5]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_5]
