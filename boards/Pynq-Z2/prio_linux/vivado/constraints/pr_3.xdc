create_pblock pblock_pr_3
resize_pblock pblock_pr_3 -add {SLICE_X68Y0:SLICE_X83Y49 DSP48_X3Y0:DSP48_X3Y19}
resize_pblock pblock_pr_3 -add {SLICE_X50Y0:SLICE_X63Y49}
resize_pblock pblock_pr_3 -add {SLICE_X32Y0:SLICE_X47Y49 DSP48_X2Y0:DSP48_X2Y19 RAMB18_X2Y0:RAMB18_X3Y19 RAMB36_X2Y0:RAMB36_X3Y9}
add_cells_to_pblock pblock_pr_3 [get_cells -quiet [list prio_linux_i/pr_3]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_3]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_3]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_3]
