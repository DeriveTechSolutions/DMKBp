  #include "port_ui.h"

//#define ST 37

#define ST 37


/*ui* ui_new(void (*ui_menu_callback)(void))
{
    ui *p;
    p = (ui *)malloc(sizeof(ui));
    p->menu_state = 0;
    p->submenu_state = 0;
    p->demo_title_delay = 0;
    p->demo_description_delay = 0;
    p->state_internal = 0;
  //  p->menu = ui_menu_callback;

    uint8_t index;
    for(index = 0; index < 3; index++)
    {
      //  p->button_state[index] = 5;
    //    p->button_state_prev[index] = 4;
  //      p->button_event[index] = 0;
    }
    return p;
}*/


#define ENTRY 0
#define DO 1
#define EXIT 2
/*
uint8_t ui_tasks(ui *p)
{

    uint16_t index;
    static uint8_t entrydoexit = ENTRY;

    p->btn_left_event = 0;
    p->btn_right_event = 0;

    static uint8_t state_oneshot[]={0,0,0};
    for(index = 0; index < UI_BTN_MAX; index++) {
        switch(state_oneshot[index]) {
            case 0:
                if(p->button_state[index] == 0x01 ) {
                    state_oneshot[index] = 1;
                }
                break;
            case 1:
                if(p->button_state[index] == 0x02 ) {
                    state_oneshot[index] = 2;
                }
                break;
            case 2:
                p->button_event[index] = 1;
                state_oneshot[index] = 0;
                break;
        }
    }

    switch(p->state_internal)
    {
        case UI_START:
            p->state_internal = DISPLAY_DATA;
            for(index = 0; index < 3; index++) { p->button_event[index] = 0;}
            break;
        case DISPLAY_DATA:
            if(entrydoexit == ENTRY)
            {
                p->p_lcd->efficient_raster_override = 0;
                p->p_lcd->scroll_zero = 0;
                p->p_lcd->printf_disabled = 0;
                p->p_lcd->icons_disabled = 0;
                lcd_scroll_indices_set(p->p_lcd,0,LCD_CANVAS_MAX_X);
                entrydoexit = DO;
            }
            if(entrydoexit == DO)
            {
                if(p->submenu_state == 0){
                    ui_callback_set_scroll_speed(p->Fosc, p->refresh_rate_while_scrolling);
                }
                else
                {
                    ui_callback_set_scroll_speed(p->Fosc, p->refresh_rate_not_scrolling);
                }
                if(p->submenu_state != 0 && p->menu_state == 0)
                {
                    if(ui_btn_pressed(p,UI_LEFT)){p->btn_left_event = 1;}
                    if(ui_btn_pressed(p,UI_CIRCLE)){p->state_internal = SUB_MENU_TRANSITION;entrydoexit=EXIT;}
                    if(ui_btn_pressed(p,UI_RIGHT)){p->btn_right_event = 1;}
                }
                else
                {
                    if(ui_btn_pressed(p,UI_LEFT)){p->state_internal = TRANSITION_LEFT;entrydoexit=EXIT;}
                    if(ui_btn_pressed(p,UI_CIRCLE)){p->state_internal = SUB_MENU_TRANSITION;entrydoexit=EXIT;}
                    if(ui_btn_pressed(p,UI_RIGHT)){p->state_internal = TRANSITION_RIGHT;entrydoexit=EXIT;}
                }
            }
            if(entrydoexit == EXIT)
            {
                entrydoexit = ENTRY;
            }
            break;

        case TRANSITION_RIGHT:
            if(entrydoexit == ENTRY)
            {

                ui_callback_set_scroll_speed(p->Fosc, p->refresh_rate_while_scrolling);
                p->p_lcd->efficient_raster_override = 1;
                p->submenu_state = 0;
                p->p_lcd->printf_disabled = 1;
                p->p_lcd->icons_disabled = 1;
                p->p_lcd->iconALL = 0;

                p->menu_state++;
                p->menu_state = math_virtual_index(p->menu_state_max,0,p->menu_state);
                p->menu();

                p->p_lcd->cursor = LCD_MAX_X;
                strcpy(p->p_lcd->buffer,p->demo_title);
                for(index = LCD_MAX_X; index < lcd_msg_length_px(p->p_lcd)+LCD_MAX_X; index++) {
                    p->p_lcd->canvas[index].byte=0;
                }
                lcd_canvas_printf(p->p_lcd);
                entrydoexit = DO;
            }
            if(entrydoexit == DO)
            {
                if(p->button_event[UI_LEFT])
                {
                    p->state_internal = TRANSITION_RIGHT_CANCEL;
                    entrydoexit=EXIT;
                }
                lcd_scroll_left(p->p_lcd);
                if(p->p_lcd->canvas[p->p_lcd->scroll_indices[0]].stop)
                {

                    p->state_internal = DISPLAY_DEMO_TITLE;
                    entrydoexit = EXIT;
                }
            }
            if(entrydoexit == EXIT)
            {
                entrydoexit = ENTRY;
            }
            break;
        case TRANSITION_RIGHT_CANCEL:
            lcd_scroll_right(p->p_lcd);
            if(p->p_lcd->canvas[p->p_lcd->scroll_indices[0]].stop)
            {

                p->p_lcd->scroll_zero = 0;

                lcd_scroll_indices_set(p->p_lcd,0,LCD_CANVAS_MAX_X);

                p->menu_state--;
                p->menu_state = math_virtual_index(p->menu_state_max,0,p->menu_state);
                p->menu();
                entrydoexit = ENTRY;
                p->state_internal = DISPLAY_DATA;
            }
            break;
        case TRANSITION_LEFT:
            if(entrydoexit == ENTRY)
            {
                ui_callback_set_scroll_speed(p->Fosc, p->refresh_rate_while_scrolling);
                p->p_lcd->iconALL = 0;
                p->p_lcd->efficient_raster_override = 1;
                p->submenu_state = 0;
                p->p_lcd->icons_disabled = 1;
                p->p_lcd->printf_disabled = 1;

                p->menu_state--;
                p->menu_state = math_virtual_index(p->menu_state_max,0,p->menu_state);
                p->menu();

                p->p_lcd->cursor = LCD_CANVAS_MAX_X - lcd_msg_length_px(p->p_lcd);
                if(lcd_msg_length_px(p->p_lcd) < LCD_MAX_X)
                    p->p_lcd->cursor = LCD_CANVAS_MAX_X - LCD_MAX_X;
                strcpy(p->p_lcd->buffer,p->demo_title);
                for(index = LCD_MAX_X; index < lcd_msg_length_px(p->p_lcd)+LCD_MAX_X; index++) {
                    p->p_lcd->canvas[index].byte=0;
                }
                lcd_canvas_printf(p->p_lcd);

                entrydoexit = DO;
            }
            if(entrydoexit == DO)
            {
                lcd_scroll_right(p->p_lcd);
                if(p->p_lcd->canvas[p->p_lcd->scroll_indices[0]].stop)
                {

                    entrydoexit = EXIT;
                }
            }
            if(entrydoexit == EXIT)
            {
                p->state_internal = DISPLAY_DEMO_TITLE;
                entrydoexit = ENTRY;
            }
            break;
        case SUB_MENU_TRANSITION:
            p->submenu_state++;
            p->menu();
            if(p->submenu_state == p->submenu_state_max)
            {
                p->submenu_state = 0;
            }
            entrydoexit = ENTRY;
            p->state_internal = DISPLAY_DATA;

            for(index = 0; index < 3; index++) { p->button_event[index] = 0;}
            break;
        case DISPLAY_DEMO_TITLE:
            if(entrydoexit == ENTRY)
            {
                entrydoexit = DO;
            }
            if(entrydoexit == DO)
            {
                if(p->demo_title_delay++ == TITLE_DELAY)
                {
                    lcd_canvas_clr(p->p_lcd);

                    p->demo_title_delay = 0;

                    p->p_lcd->scroll_zero = 0;

                    lcd_scroll_indices_set(p->p_lcd,0,LCD_CANVAS_MAX_X);

                    strcpy(p->p_lcd->buffer,p->demo_description);
                    p->p_lcd->cursor = 0;
                    lcd_canvas_printf(p->p_lcd);


                    entrydoexit = EXIT;
                }
            }
            if(entrydoexit == EXIT)
            {
                p->state_internal = DISPLAY_DEMO_DESCRIPTION;
                // Nothin'
            }
            break;
        case DISPLAY_DEMO_DESCRIPTION:
            if(p->demo_description_delay++ == TITLE_DELAY) {
                p->demo_description_delay = 0;
                p->p_lcd->printf_disabled = 0;
                p->p_lcd->icons_disabled = 0;
                entrydoexit = ENTRY;
                p->state_internal = DISPLAY_DATA;
            }
            break;
        default:
            break;
    }

    p->menu_state = math_virtual_index(p->menu_state_max,0,p->menu_state);

    p->menu();
    return p->state_internal;

}*/




#define ENTRY 0
#define DO 1
#define EXIT 2
