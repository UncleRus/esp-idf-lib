#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mp2660.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *MAIN_LOG_TAG = "Main";

void task(void *pvParameters)
{
    i2c_dev_t charger;
    memset(&charger, 0, sizeof(i2c_dev_t));

    i2cdev_init();

    esp_err_t err = mp2660_init_desc(&charger, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Cannot init MP2660: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    do
    {
        mp2660_fault_t fault;
        err = mp2660_get_fault(&charger, &fault);

        ESP_LOGI(MAIN_LOG_TAG, "Result reading fault register: %s", esp_err_to_name(err));
    }
    while (err != ESP_OK);

    for (;;)
    {
        mp2660_input_source_t input_source;
        err = mp2660_get_input_source(&charger, &input_source);
        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 input source register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, EnHIZ: %d, VInMin3: %d, VInMin2: %d, VInMin1: %d, VInMin0: %d, IInLim_2: %d, IInLim_1: %d, IInLim_0: %d", input_source.register_data.reg,
                input_source.data_fields.en_hiz, input_source.data_fields.v_in_min_3, input_source.data_fields.v_in_min_2, input_source.data_fields.v_in_min_1, input_source.data_fields.v_in_min_0,
                input_source.data_fields.i_in_lim_2, input_source.data_fields.i_in_lim_1, input_source.data_fields.i_in_lim_0);
        }

        mp2660_power_on_conf_t pwr_conf;
        err = mp2660_get_pwr_on_conf(&charger, &pwr_conf);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 power configuration register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, UVLO 0: %d, UVLO 1: %d, UVLO 2: %d, CEB: %d, WatchdogTimer: %d, RegisterReset: %d", pwr_conf.register_data.reg, pwr_conf.data_fields.v_batt_uvlo_0,
                pwr_conf.data_fields.v_batt_uvlo_1, pwr_conf.data_fields.v_batt_uvlo_2, pwr_conf.data_fields.ceb, pwr_conf.data_fields.i2c_watchdog_timer, pwr_conf.data_fields.reg_reset);
        }

        mp2660_charge_current_ctrl_t charge_current_ctrl;
        err = mp2660_get_chrg_current_ctrl(&charger, &charge_current_ctrl);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 charge current control register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, ICC 0: %d, ICC 1: %d, ICC 2: %d, ICC 3: %d, ICC 4: %d", charge_current_ctrl.register_data.reg, charge_current_ctrl.data_fields.icc_0,
                charge_current_ctrl.data_fields.icc_1, charge_current_ctrl.data_fields.icc_2, charge_current_ctrl.data_fields.icc_3, charge_current_ctrl.data_fields.icc_4);
        }

        mp2660_pre_charge_term_current_t pre_chrg_term_current;
        err = mp2660_get_pre_chrg_term_current(&charger, &pre_chrg_term_current);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 pre charge/termination current control register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, DSCHG 0: %d, DSCHG 1: %d, DSCHG 2: %d, DSCHG 3: %d, IPre 0: %d, IPre 1: %d", pre_chrg_term_current.register_data.reg,
                pre_chrg_term_current.data_fields.i_dschg_0, pre_chrg_term_current.data_fields.i_dschg_1, pre_chrg_term_current.data_fields.i_dschg_2, pre_chrg_term_current.data_fields.i_dschg_3,
                pre_chrg_term_current.data_fields.i_pre_0, pre_chrg_term_current.data_fields.i_pre_1);
        }

        mp2660_charge_voltage_ctrl_t chrg_voltage_ctrl;
        err = mp2660_get_chrg_voltage_control(&charger, &chrg_voltage_ctrl);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 charge voltage control register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, VBattReg 0: %d, VBattReg 1: %d, VBattReg 2: %d, VBattReg 3: %d, VBattReg 4: %d, VBattReg 5: %d, VBatt pre: %d, VBatt rech: %d",
                chrg_voltage_ctrl.register_data.reg, chrg_voltage_ctrl.data_fields.v_bat_reg_0, chrg_voltage_ctrl.data_fields.v_bat_reg_1, chrg_voltage_ctrl.data_fields.v_bat_reg_2,
                chrg_voltage_ctrl.data_fields.v_bat_reg_3, chrg_voltage_ctrl.data_fields.v_bat_reg_4, chrg_voltage_ctrl.data_fields.v_bat_reg_5, chrg_voltage_ctrl.data_fields.v_batt_pre,
                chrg_voltage_ctrl.data_fields.v_batt_rech);
        }

        mp2660_charge_term_timer_ctrl_t charge_term_timer_ctrl;
        err = mp2660_get_chrg_term_timer_control(&charger, &charge_term_timer_ctrl);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 charge termination / timer control register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, EnTerm: %d, WatchDog 0: %d, WatchDog 1: %d, EnTimer: %d, ChgTimer 0: %d, ChgTimer 1: %d, TermTmr: %d", charge_term_timer_ctrl.register_data.reg,
                charge_term_timer_ctrl.data_fields.en_term, charge_term_timer_ctrl.data_fields.watchdog_0, charge_term_timer_ctrl.data_fields.watchdog_1, charge_term_timer_ctrl.data_fields.en_timer,
                charge_term_timer_ctrl.data_fields.chg_timer_0, charge_term_timer_ctrl.data_fields.chg_timer_1, charge_term_timer_ctrl.data_fields.term_tmr);
        }

        mp2660_misc_op_ctrl_t misc_op_ctrl;
        err = mp2660_get_misc_op_control(&charger, &misc_op_ctrl);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 misc operation control register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, TMR2XEn: %d, FetDis: %d, EnNtc: %d, TJReg 0: %d, TJReg 1: %d", misc_op_ctrl.register_data.reg, misc_op_ctrl.data_fields.tmr2x_en,
                misc_op_ctrl.data_fields.fet_dis, misc_op_ctrl.data_fields.en_ntc, misc_op_ctrl.data_fields.tj_reg_0, misc_op_ctrl.data_fields.tj_reg_1);
        }

        mp2660_sys_status_t sys_status;
        err = mp2660_get_sys_status(&charger, &sys_status);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 system status register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, Rev 1: %d, Rev 0: %d, ChgStat 1: %d, ChgStat 0: %d, PPMStat: %d, PGStat: %d, ThermStat: %d", sys_status.register_data.reg,
                sys_status.data_fields.rev_1, sys_status.data_fields.rev_0, sys_status.data_fields.chg_stat_1, sys_status.data_fields.chg_stat_0, sys_status.data_fields.ppm_stat,
                sys_status.data_fields.pg_stat, sys_status.data_fields.therm_stat);
        }

        mp2660_fault_t fault;
        err = mp2660_get_fault(&charger, &fault);

        if (err != ESP_OK)
        {
            ESP_LOGI(MAIN_LOG_TAG, "Cannot read MP2660 fault register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, WatchdogFault: %d, VinFault: %d, ThemSd: %d, BatFault 0: %d, StmrFault: %d", fault.register_data.reg, fault.data_fields.watchdog_fault,
                fault.data_fields.vin_fault, fault.data_fields.them_sd, fault.data_fields.bat_fault, fault.data_fields.stmr_fault);
        }

        ESP_LOGI(MAIN_LOG_TAG, "Done.");

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
