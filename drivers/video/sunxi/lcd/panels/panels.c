#include "panels.h"
#include "default_panel.h"
#include "tft720x1280.h"

extern __lcd_panel_t tft720x1280_panel;
extern __lcd_panel_t vvx10f004b00_panel;
extern __lcd_panel_t lp907qx_panel;

__lcd_panel_t* panel_array[] = {
	&default_panel,
	&tft720x1280_panel,
	&vvx10f004b00_panel,
	&lp907qx_panel,
	/* add new panel below */

	NULL,
};

