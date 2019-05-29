// trans complete int code:
} else if (status & I2C_TRANS_COMPLETE_INT_ST_M) {
            I2C[i2c_num]->int_clr.trans_complete = 1;
            if (p_i2c->mode == I2C_MODE_SLAVE) {
                int rx_fifo_cnt = I2C[i2c_num]->status_reg.rx_fifo_cnt;
                for (idx = 0; idx < rx_fifo_cnt; idx++) {
                    p_i2c->data_buf[idx] = I2C[i2c_num]->fifo_data.data;
                }
                xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);
                I2C[i2c_num]->int_clr.rx_fifo_full = 1;
            } else {
                // add check for unexcepted situations caused by noise.
                if (p_i2c->status != I2C_STATUS_ACK_ERROR && p_i2c->status != I2C_STATUS_IDLE) {
                    i2c_master_cmd_begin_static(i2c_num);
                }
            }

// rxfifo full int code:
} else if (status & I2C_RXFIFO_FULL_INT_ST_M) {
            int rx_fifo_cnt = I2C[i2c_num]->status_reg.rx_fifo_cnt;
            for (idx = 0; idx < rx_fifo_cnt; idx++) {
                p_i2c->data_buf[idx] = I2C[i2c_num]->fifo_data.data;
            }
            xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);
            I2C[i2c_num]->int_clr.rx_fifo_full = 1;