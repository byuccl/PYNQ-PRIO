create_pblock pblock_pr_2
resize_pblock pblock_pr_2 -add {SLICE_X0Y0:SLICE_X31Y49 DSP48_X0Y0:DSP48_X1Y19 RAMB18_X0Y0:RAMB18_X1Y19 RAMB36_X0Y0:RAMB36_X1Y9}
add_cells_to_pblock pblock_pr_2 [get_cells -quiet [list prio_linux_i/pr_2]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_2]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_2]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_2]
