// ------------------------ Disclaimer -----------------------
// No warranty of correctness, synthesizability or 
// functionality of this code is given.
// Use this code under your own risk.
// When using this code, copy this disclaimer at the top of 
// Your file
//
// (c) Luca Hanel 2020
//
// ------------------------------------------------------------
//
// Module name: gpio_module
// 
// Functionality: GPIO module
//
// ------------------------------------------------------------

`include "gpio_inc.sv"

module gpio_module#(
    parameter N_GPIOS = 8
)(
    input logic                 clk,
    input logic                 rstn_i,
    output logic [N_GPIOS-1:0]  dir_o,
    output logic [N_GPIOS-1:0]  val_o,
    input logic [N_GPIOS-1:0]   val_i,
    output logic [N_GPIOS-1:0]  irq_o,
    wb_bus_t.slave              wb_bus
);

logic [4:0][31:0] gpio_regs_n;
logic [4:0][31:0] gpio_regs_q;

logic [31:0] gpio_do;

logic [N_GPIOS-1:0] gpio_vi;
logic [N_GPIOS-1:0] gpio_vo;

assign dir_o = gpio_regs_q[`GPIO_DIR][N_GPIOS-1:0];
assign wb_bus.wb_dat_sm = gpio_do;

genvar ii;
for(ii = 0; ii < N_GPIOS; ii = ii + 1) begin
    assign gpio_vi[ii] = (gpio_regs_q[`GPIO_INV][ii]) ? !val_i[ii] : val_i[ii];
    assign gpio_vo[ii] = (gpio_regs_q[`GPIO_INV][ii]) ? !gpio_regs_q[`GPIO_VAL][ii] : gpio_regs_q[`GPIO_VAL][ii];
    assign val_o[ii] = (gpio_regs_q[`GPIO_DIR][ii]) ? gpio_vo[ii] : 1'b0;
end

always_comb
begin
    gpio_regs_n = gpio_regs_q;
    wb_bus.wb_err = 1'b0;
    wb_bus.wb_ack = 1'b0;
    irq_o  = 'b0;
    gpio_do = 'b0;

    // WB slave
    if(wb_bus.wb_cyc && wb_bus.wb_stb) begin
        wb_bus.wb_ack = 1'b1;
        if(wb_bus.wb_adr > 32'h10)
            // If the address is out of bounds, return error
            wb_bus.wb_err = 1'b1;
        else begin
            if(wb_bus.wb_we) begin
                // Writing
                gpio_regs_n[wb_bus.wb_adr[4:2]] = wb_bus.wb_dat_ms;
            end else begin
                // Reading (only supports full 32 bit reading)
                gpio_do = gpio_regs_q[wb_bus.wb_adr[4:2]];
            end
        end
    end
    
    // GPIO
    for(int i = 0; i < N_GPIOS; i = i + 1) begin
        if(!(gpio_regs_q[`GPIO_DIR][i])) begin
            // Pin is input pin
            gpio_regs_n[`GPIO_VAL][i] = gpio_vi[i];

            // Interrupts
            if(gpio_regs_q[`GPIO_INT_EN][i]) begin
                case(gpio_regs_q[`GPIO_INT_T][i])
                    `GPIO_INT_T_RISE: begin // Trigger on rising edge
                        if(!gpio_regs_q[`GPIO_VAL][i] && gpio_vi[i])
                            irq_o[i] = 1'b1;
                    end

                    `GPIO_INT_T_EDGE: begin // Trigger on any edge
                        if(gpio_regs_q[`GPIO_VAL][i] != gpio_vi[i])
                            irq_o[i] = 1'b1;
                    end
                endcase
            end
        end
    end
end

always_ff @(posedge clk, negedge rstn_i)
begin
    if(!rstn_i) begin
        gpio_regs_q[`GPIO_DIR] <= 'b0;
        gpio_regs_q[`GPIO_VAL] <= 'b0;
        gpio_regs_q[`GPIO_INV] <= 'b0;
        gpio_regs_q[`GPIO_INT_EN] <= 'b0;
        gpio_regs_q[`GPIO_INT_T] <= 'b0;
    end else begin
        gpio_regs_q[`GPIO_DIR] <= gpio_regs_n[`GPIO_DIR];
        gpio_regs_q[`GPIO_VAL] <= gpio_regs_n[`GPIO_VAL];
        gpio_regs_q[`GPIO_INV] <= gpio_regs_n[`GPIO_INV];
        gpio_regs_q[`GPIO_INT_EN] <= gpio_regs_n[`GPIO_INT_EN];
        gpio_regs_q[`GPIO_INT_T] <= gpio_regs_n[`GPIO_INT_T];
    end
end 

endmodule