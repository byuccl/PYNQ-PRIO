
################################################################
# This is a generated script based on design: rm_spi
#
# Though there are limitations about the generated script,
# the main purpose of this utility is to make learning
# IP Integrator Tcl commands easier.
################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2024.2
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   catch {common::send_msg_id "BD_TCL-109" "ERROR" "This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."}

   return 1
}

################################################################
# START
################################################################

# To test this script, run the following commands from Vivado Tcl console:
# source rm_spi_script.tcl

# If there is no project opened, this script will create a
# project, but make sure you do not have an existing project
# <./myproj/project_1.xpr> in the current working folder.

set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
   create_project project_1 myproj -part xczu7ev-ffvc1156-2-e
   set_property BOARD_PART xilinx.com:zcu104:part0:1.0 [current_project]
}


# CHANGE DESIGN NAME HERE
variable design_name
set design_name spi

# If you do not already have an existing IP Integrator design open,
# you can create a design using the following command:
#    create_bd_design $design_name

# Creating design if needed
set errMsg ""
set nRet 0

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

if { ${design_name} eq "" } {
   # USE CASES:
   #    1) Design_name not set

   set errMsg "Please set the variable <design_name> to a non-empty value."
   set nRet 1

} elseif { ${cur_design} ne "" && ${list_cells} eq "" } {
   # USE CASES:
   #    2): Current design opened AND is empty AND names same.
   #    3): Current design opened AND is empty AND names diff; design_name NOT in project.
   #    4): Current design opened AND is empty AND names diff; design_name exists in project.

   if { $cur_design ne $design_name } {
      common::send_msg_id "BD_TCL-001" "INFO" "Changing value of <design_name> from <$design_name> to <$cur_design> since current design is empty."
      set design_name [get_property NAME $cur_design]
   }
   common::send_msg_id "BD_TCL-002" "INFO" "Constructing design in IPI design <$cur_design>..."

} elseif { ${cur_design} ne "" && $list_cells ne "" && $cur_design eq $design_name } {
   # USE CASES:
   #    5) Current design opened AND has components AND same names.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 1
} elseif { [get_files -quiet ${design_name}.bd] ne "" } {
   # USE CASES:
   #    6) Current opened design, has components, but diff names, design_name exists in project.
   #    7) No opened design, design_name exists in project.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 2

} else {
   # USE CASES:
   #    8) No opened design, design_name not in project.
   #    9) Current opened design, has components, but diff names, design_name not in project.

   common::send_msg_id "BD_TCL-003" "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

   common::send_msg_id "BD_TCL-004" "INFO" "Making design <$design_name> as current_bd_design."
   current_bd_design $design_name

}

common::send_msg_id "BD_TCL-005" "INFO" "Currently the variable <design_name> is equal to \"$design_name\"."

if { $nRet != 0 } {
   catch {common::send_msg_id "BD_TCL-114" "ERROR" $errMsg}
   return $nRet
}

set bCheckIPsPassed 1
##################################################################
# CHECK IPs
##################################################################
set bCheckIPs 1
if { $bCheckIPs == 1 } {
   set list_check_ips "\
xilinx.com:ip:axi_quad_spi:3.2\
xilinx.com:ip:xlconcat:2.1\
xilinx.com:ip:xlslice:1.0\
"

   set list_ips_missing ""
   common::send_msg_id "BD_TCL-006" "INFO" "Checking if the following IPs exist in the project's IP catalog: $list_check_ips ."

   foreach ip_vlnv $list_check_ips {
      set ip_obj [get_ipdefs -all $ip_vlnv]
      if { $ip_obj eq "" } {
         lappend list_ips_missing $ip_vlnv
      }
   }

   if { $list_ips_missing ne "" } {
      catch {common::send_msg_id "BD_TCL-115" "ERROR" "The following IPs are not found in the IP Catalog:\n  $list_ips_missing\n\nResolution: Please add the repository containing the IP(s) to the project." }
      set bCheckIPsPassed 0
   }

}

if { $bCheckIPsPassed != 1 } {
  common::send_msg_id "BD_TCL-1003" "WARNING" "Will not continue with creation of design due to the error(s) above."
  return 3
}

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  variable script_folder
  variable design_name

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_msg_id "BD_TCL-100" "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_msg_id "BD_TCL-101" "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports

  # Create ports
  set S_AXI_araddr [ create_bd_port -dir I -from 8 -to 0 S_AXI_araddr ]
  set S_AXI_arready [ create_bd_port -dir O S_AXI_arready ]
  set S_AXI_arvalid [ create_bd_port -dir I S_AXI_arvalid ]
  set S_AXI_awaddr [ create_bd_port -dir I -from 8 -to 0 S_AXI_awaddr ]
  set S_AXI_awready [ create_bd_port -dir O S_AXI_awready ]
  set S_AXI_awvalid [ create_bd_port -dir I S_AXI_awvalid ]
  set S_AXI_bready [ create_bd_port -dir I S_AXI_bready ]
  set S_AXI_bresp [ create_bd_port -dir O -from 1 -to 0 S_AXI_bresp ]
  set S_AXI_bvalid [ create_bd_port -dir O S_AXI_bvalid ]
  set S_AXI_rdata [ create_bd_port -dir O -from 31 -to 0 S_AXI_rdata ]
  set S_AXI_rready [ create_bd_port -dir I S_AXI_rready ]
  set S_AXI_rresp [ create_bd_port -dir O -from 1 -to 0 S_AXI_rresp ]
  set S_AXI_rvalid [ create_bd_port -dir O S_AXI_rvalid ]
  set S_AXI_wdata [ create_bd_port -dir I -from 31 -to 0 S_AXI_wdata ]
  set S_AXI_wready [ create_bd_port -dir O S_AXI_wready ]
  set S_AXI_wstrb [ create_bd_port -dir I -from 3 -to 0 S_AXI_wstrb ]
  set S_AXI_wvalid [ create_bd_port -dir I S_AXI_wvalid ]
  set ip2intc_irpt [ create_bd_port -dir O -type intr ip2intc_irpt ]
  set pr_4bits_tri_i [ create_bd_port -dir I -from 3 -to 0 pr_4bits_tri_i ]
  set pr_4bits_tri_o [ create_bd_port -dir O -from 3 -to 0 pr_4bits_tri_o ]
  set pr_4bits_tri_t [ create_bd_port -dir O -from 3 -to 0 pr_4bits_tri_t ]
  set s_axi_aclk [ create_bd_port -dir I -type clk s_axi_aclk ]
  set_property -dict [ list \
   CONFIG.FREQ_HZ {99999001} \
 ] $s_axi_aclk
  set s_axi_aresetn [ create_bd_port -dir I -type rst s_axi_aresetn ]

  # Create instance: axi_quad_spi_0, and set properties
  set axi_quad_spi_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_quad_spi:3.2 axi_quad_spi_0 ]

  # Create instance: xlconcat_0, and set properties
  set xlconcat_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_0 ]
  set_property -dict [ list \
   CONFIG.NUM_PORTS {4} \
 ] $xlconcat_0

  # Create instance: xlconcat_1, and set properties
  set xlconcat_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_1 ]
  set_property -dict [ list \
   CONFIG.NUM_PORTS {4} \
 ] $xlconcat_1

  # Create instance: xlslice_0, and set properties
  set xlslice_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_0 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {0} \
   CONFIG.DIN_TO {0} \
   CONFIG.DIN_WIDTH {4} \
 ] $xlslice_0

  # Create instance: xlslice_1, and set properties
  set xlslice_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_1 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {1} \
   CONFIG.DIN_TO {1} \
   CONFIG.DIN_WIDTH {4} \
   CONFIG.DOUT_WIDTH {1} \
 ] $xlslice_1

  # Create instance: xlslice_2, and set properties
  set xlslice_2 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_2 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {2} \
   CONFIG.DIN_TO {2} \
   CONFIG.DIN_WIDTH {4} \
 ] $xlslice_2

  # Create instance: xlslice_3, and set properties
  set xlslice_3 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_3 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {3} \
   CONFIG.DIN_TO {3} \
   CONFIG.DIN_WIDTH {4} \
 ] $xlslice_3

  # Create instance: xlslice_4, and set properties
  set xlslice_4 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_4 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {6} \
   CONFIG.DIN_TO {0} \
   CONFIG.DIN_WIDTH {9} \
   CONFIG.DOUT_WIDTH {7} \
 ] $xlslice_4

  # Create instance: xlslice_5, and set properties
  set xlslice_5 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_5 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {6} \
   CONFIG.DIN_TO {0} \
   CONFIG.DIN_WIDTH {9} \
   CONFIG.DOUT_WIDTH {7} \
 ] $xlslice_5

  # Create port connections
  connect_bd_net -net Net [get_bd_ports S_AXI_awaddr] [get_bd_pins xlslice_5/Din]
  connect_bd_net -net S_AXI_arvalid_1 [get_bd_ports S_AXI_arvalid] [get_bd_pins axi_quad_spi_0/s_axi_arvalid]
  connect_bd_net -net S_AXI_awvalid_1 [get_bd_ports S_AXI_awvalid] [get_bd_pins axi_quad_spi_0/s_axi_awvalid]
  connect_bd_net -net S_AXI_bready_1 [get_bd_ports S_AXI_bready] [get_bd_pins axi_quad_spi_0/s_axi_bready]
  connect_bd_net -net S_AXI_rready_1 [get_bd_ports S_AXI_rready] [get_bd_pins axi_quad_spi_0/s_axi_rready]
  connect_bd_net -net S_AXI_wdata_1 [get_bd_ports S_AXI_wdata] [get_bd_pins axi_quad_spi_0/s_axi_wdata]
  connect_bd_net -net S_AXI_wstrb_1 [get_bd_ports S_AXI_wstrb] [get_bd_pins axi_quad_spi_0/s_axi_wstrb]
  connect_bd_net -net S_AXI_wvalid_1 [get_bd_ports S_AXI_wvalid] [get_bd_pins axi_quad_spi_0/s_axi_wvalid]
  connect_bd_net -net axi_quad_spi_0_io0_o [get_bd_pins axi_quad_spi_0/io0_o] [get_bd_pins xlconcat_0/In0]
  connect_bd_net -net axi_quad_spi_0_io0_t [get_bd_pins axi_quad_spi_0/io0_t] [get_bd_pins xlconcat_1/In0]
  connect_bd_net -net axi_quad_spi_0_io1_o [get_bd_pins axi_quad_spi_0/io1_o] [get_bd_pins xlconcat_0/In1]
  connect_bd_net -net axi_quad_spi_0_io1_t [get_bd_pins axi_quad_spi_0/io1_t] [get_bd_pins xlconcat_1/In1]
  connect_bd_net -net axi_quad_spi_0_ip2intc_irpt [get_bd_ports ip2intc_irpt] [get_bd_pins axi_quad_spi_0/ip2intc_irpt]
  connect_bd_net -net axi_quad_spi_0_s_axi_arready [get_bd_ports S_AXI_arready] [get_bd_pins axi_quad_spi_0/s_axi_arready]
  connect_bd_net -net axi_quad_spi_0_s_axi_awready [get_bd_ports S_AXI_awready] [get_bd_pins axi_quad_spi_0/s_axi_awready]
  connect_bd_net -net axi_quad_spi_0_s_axi_bresp [get_bd_ports S_AXI_bresp] [get_bd_pins axi_quad_spi_0/s_axi_bresp]
  connect_bd_net -net axi_quad_spi_0_s_axi_bvalid [get_bd_ports S_AXI_bvalid] [get_bd_pins axi_quad_spi_0/s_axi_bvalid]
  connect_bd_net -net axi_quad_spi_0_s_axi_rdata [get_bd_ports S_AXI_rdata] [get_bd_pins axi_quad_spi_0/s_axi_rdata]
  connect_bd_net -net axi_quad_spi_0_s_axi_rresp [get_bd_ports S_AXI_rresp] [get_bd_pins axi_quad_spi_0/s_axi_rresp]
  connect_bd_net -net axi_quad_spi_0_s_axi_rvalid [get_bd_ports S_AXI_rvalid] [get_bd_pins axi_quad_spi_0/s_axi_rvalid]
  connect_bd_net -net axi_quad_spi_0_s_axi_wready [get_bd_ports S_AXI_wready] [get_bd_pins axi_quad_spi_0/s_axi_wready]
  connect_bd_net -net axi_quad_spi_0_sck_o [get_bd_pins axi_quad_spi_0/sck_o] [get_bd_pins xlconcat_0/In2]
  connect_bd_net -net axi_quad_spi_0_sck_t [get_bd_pins axi_quad_spi_0/sck_t] [get_bd_pins xlconcat_1/In2]
  connect_bd_net -net axi_quad_spi_0_ss_o [get_bd_pins axi_quad_spi_0/ss_o] [get_bd_pins xlconcat_0/In3]
  connect_bd_net -net axi_quad_spi_0_ss_t [get_bd_pins axi_quad_spi_0/ss_t] [get_bd_pins xlconcat_1/In3]
  connect_bd_net -net pr_4bits_tri_i_1 [get_bd_ports pr_4bits_tri_i] [get_bd_pins xlslice_0/Din] [get_bd_pins xlslice_1/Din] [get_bd_pins xlslice_2/Din] [get_bd_pins xlslice_3/Din]
  connect_bd_net -net s_axi_aclk_1 [get_bd_ports s_axi_aclk] [get_bd_pins axi_quad_spi_0/ext_spi_clk] [get_bd_pins axi_quad_spi_0/s_axi_aclk]
  connect_bd_net -net s_axi_araddr_1 [get_bd_ports S_AXI_araddr] [get_bd_pins xlslice_4/Din]
  connect_bd_net -net s_axi_aresetn_1 [get_bd_ports s_axi_aresetn] [get_bd_pins axi_quad_spi_0/s_axi_aresetn]
  connect_bd_net -net xlconcat_0_dout [get_bd_ports pr_4bits_tri_o] [get_bd_pins xlconcat_0/dout]
  connect_bd_net -net xlconcat_1_dout [get_bd_ports pr_4bits_tri_t] [get_bd_pins xlconcat_1/dout]
  connect_bd_net -net xlslice_0_Dout [get_bd_pins axi_quad_spi_0/io0_i] [get_bd_pins xlslice_0/Dout]
  connect_bd_net -net xlslice_1_Dout [get_bd_pins axi_quad_spi_0/io1_i] [get_bd_pins xlslice_1/Dout]
  connect_bd_net -net xlslice_2_Dout [get_bd_pins axi_quad_spi_0/sck_i] [get_bd_pins xlslice_2/Dout]
  connect_bd_net -net xlslice_3_Dout [get_bd_pins axi_quad_spi_0/ss_i] [get_bd_pins xlslice_3/Dout]
  connect_bd_net -net xlslice_4_Dout [get_bd_pins axi_quad_spi_0/s_axi_araddr] [get_bd_pins xlslice_4/Dout]
  connect_bd_net -net xlslice_5_Dout [get_bd_pins axi_quad_spi_0/s_axi_awaddr] [get_bd_pins xlslice_5/Dout]

  # Create address segments


  # Restore current instance
  current_bd_instance $oldCurInst

  validate_bd_design
  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""
