create_pblock pblock_pr_0
resize_pblock pblock_pr_0 -add {SLICE_X89Y330:SLICE_X111Y359 DSP48E2_X12Y132:DSP48E2_X13Y143 RAMB18_X3Y132:RAMB18_X4Y143 RAMB36_X3Y66:RAMB36_X4Y71}
add_cells_to_pblock pblock_pr_0 [get_cells -quiet [list prio_i/pr0]]
set_property SNAPPING_MODE ON [get_pblocks pblock_pr_0]
