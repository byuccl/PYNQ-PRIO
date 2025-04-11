create_pblock pblock_pr_1
resize_pblock pblock_pr_1 -add {SLICE_X80Y100:SLICE_X93Y149 DSP48_X3Y40:DSP48_X3Y59 RAMB18_X4Y40:RAMB18_X4Y59 RAMB36_X4Y20:RAMB36_X4Y29}
resize_pblock pblock_pr_1 -add {SLICE_X56Y100:SLICE_X67Y149}
resize_pblock pblock_pr_1 -add {SLICE_X90Y50:SLICE_X93Y99 DSP48_X3Y20:DSP48_X3Y39}
resize_pblock pblock_pr_1 -add {SLICE_X50Y50:SLICE_X89Y99 RAMB18_X3Y20:RAMB18_X3Y39 RAMB36_X3Y10:RAMB36_X3Y19}
add_cells_to_pblock pblock_pr_1 [get_cells -quiet [list prio_linux_i/pr_1]]
set_property RESET_AFTER_RECONFIG 1 [get_pblocks pblock_pr_1]
set_property SNAPPING_MODE ROUTING [get_pblocks pblock_pr_1]
set_property PARTPIN_SPREADING 1 [get_pblocks pblock_pr_1]
