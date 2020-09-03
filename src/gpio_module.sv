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

module gpio_module#(
    parameter N_GPIOS = 8
)(
    output logic [N_GPIOS-1:0]  dir_o,
    output logic [N_GPIOS-1:0]  val_o,
    input logic [N_GPIOS-1:0]   val_i,
    output logic [N_GPIOS-1:0]  irq_o,
    apb_bus_t.slave             apb_bus
);

// registers
localparam GPIO_DIR = 0;
localparam GPIO_VAL = 1;
localparam GPIO_INV = 2;
localparam GPIO_INT_EN = 3;
localparam GPIO_INT_T = 4;
localparam GPIO_HW_DEBOUNCE = 5;

// interrupt types
localparam logic GPIO_INT_T_RISE = 1'b0;
localparam logic GPIO_INT_T_EDGE = 1'b1;

logic [5:0][31:0] gpio_regs_n;
logic [5:0][31:0] gpio_regs_q;

logic [31:0] PRDATA;
logic        PREADY;

logic [N_GPIOS-1:0] gpio_vi;
logic [6:0] gpio_dbnc[N_GPIOS-1:0];
logic [N_GPIOS-1:0] gpio_vi_dbounced;
logic [N_GPIOS-1:0] gpio_vo;

assign dir_o = gpio_regs_q[GPIO_DIR][N_GPIOS-1:0];

assign apb_bus.PRDATA = PRDATA;
assign apb_bus.PREADY = PREADY;

genvar ii;
for(ii = 0; ii < N_GPIOS; ii = ii + 1) begin
    assign gpio_vi[ii] = (gpio_regs_q[GPIO_INV][ii]) ? !val_i[ii] : val_i[ii];
    assign gpio_vo[ii] = (gpio_regs_q[GPIO_INV][ii]) ? !gpio_regs_q[GPIO_VAL][ii] : gpio_regs_q[GPIO_VAL][ii];
    assign val_o[ii] = (gpio_regs_q[GPIO_DIR][ii]) ? gpio_vo[ii] : 1'b0;
end

always_comb
begin
    PREADY = 1'b0;
    PRDATA = 'b0;

    gpio_regs_n = gpio_regs_q;
    irq_o  = 'b0;

    // APB slave
    if(apb_bus.PSEL && apb_bus.PENABLE) begin
        PREADY = 1'b1;
        if(apb_bus.PWRITE) // Write
            gpio_regs_n[apb_bus.PADDR[4:2]] = apb_bus.PWDATA;
        else // Read
            PRDATA = gpio_regs_q[apb_bus.PADDR[4:2]];
    end
    
    // GPIO
    for(int i = 0; i < N_GPIOS; i = i + 1) begin
        if(!(gpio_regs_q[GPIO_DIR][i])) begin
            // Pin is input pin
            gpio_regs_n[GPIO_VAL][i] = gpio_vi_dbounced[i];

            // Interrupts
            if(gpio_regs_q[GPIO_INT_EN][i]) begin
                case(gpio_regs_q[GPIO_INT_T][i])
                    GPIO_INT_T_RISE: begin // Trigger on rising edge
                        if(!gpio_regs_q[GPIO_VAL][i] && gpio_vi_dbounced[i])
                            irq_o[i] = 1'b1;
                    end

                    GPIO_INT_T_EDGE: begin // Trigger on any edge
                        if(gpio_regs_q[GPIO_VAL][i] != gpio_vi_dbounced[i])
                            irq_o[i] = 1'b1;
                    end
                endcase
            end
        end
    end
end

always_ff @(posedge apb_bus.PCLK, negedge apb_bus.PRESETn)
begin
    if(!apb_bus.PRESETn) begin
        gpio_regs_q[GPIO_DIR] <= 'b0;
        gpio_regs_q[GPIO_VAL] <= 'b0;
        gpio_regs_q[GPIO_INV] <= 'b0;
        gpio_regs_q[GPIO_INT_EN] <= 'b0;
        gpio_regs_q[GPIO_INT_T] <= 'b0;
        gpio_regs_q[GPIO_HW_DEBOUNCE] <= 'b0;
    end else begin
        gpio_regs_q[GPIO_DIR] <= gpio_regs_n[GPIO_DIR];
        gpio_regs_q[GPIO_VAL] <= gpio_regs_n[GPIO_VAL];
        gpio_regs_q[GPIO_INV] <= gpio_regs_n[GPIO_INV];
        gpio_regs_q[GPIO_INT_EN] <= gpio_regs_n[GPIO_INT_EN];
        gpio_regs_q[GPIO_INT_T] <= gpio_regs_n[GPIO_INT_T];
        gpio_regs_q[GPIO_HW_DEBOUNCE] <= gpio_regs_n[GPIO_HW_DEBOUNCE];

        for(int i = 0; i < N_GPIOS; i = i + 1) begin
            gpio_dbnc[i] <= {gpio_dbnc[i][5:0], gpio_vi[i]};
            case(gpio_regs_q[GPIO_HW_DEBOUNCE][2:0])
                3'h0: begin
                    gpio_vi_dbounced[i] <= gpio_vi[i];
                end

                3'h1: begin
                    if(gpio_dbnc[i][0] == 1'hf)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][0] == 1'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h2: begin
                    if(gpio_dbnc[i][1:0] == 2'hf)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][1:0] == 2'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h3: begin
                    if(gpio_dbnc[i][2:0] == 3'hf)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][2:0] == 3'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h4: begin
                    if(gpio_dbnc[i][3:0] == 4'hf)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][3:0] == 4'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h5: begin
                    if(gpio_dbnc[i][4:0] == 5'hff)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][4:0] == 5'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h6: begin
                    if(gpio_dbnc[i][5:0] == 6'hff)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][5:0] == 6'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end

                3'h7: begin
                    if(gpio_dbnc[i][6:0] == 7'hff)
                        gpio_vi_dbounced[i] <= 1'b1;
                    else if(gpio_dbnc[i][6:0] == 7'h00)
                        gpio_vi_dbounced[i] <= 1'b0;
                    else
                        gpio_vi_dbounced[i] <= gpio_vi_dbounced[i];
                end
            endcase
        end
    end
end 

endmodule