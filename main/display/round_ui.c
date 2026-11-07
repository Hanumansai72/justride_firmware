#include "round_ui.h"
#include "round_geometry.h"
#ifdef HUD_CAPTURE
#include "hud_capture_driver.h"
#else
#include "display_tft.h"
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#if CONFIG_JUSTRIDE_ROUND_AMOLED && !defined(HUD_CAPTURE)
#include "round_panel.h"
#include "assets/round_splash_logo.h"
#endif
#include "assets/call_icons.h"

#define BLACK 0x0000
#define WHITE 0xFFFF
#define SURFACE JR_RGB565(0x101512)
#define SURFACE_BORDER JR_RGB565(0x27322B)
#define GREEN JR_RGB565(0x00D99B)
#define ROUTE_GREEN JR_RGB565(0x00EF9C)
#define MUTED JR_RGB565(0xAEB6BC)
static uint16_t text_background=BLACK;
static unsigned smaller_font(unsigned size) {
    switch(size) {
    case 52:return 38;case 38:return 36;case 36:return 32;case 32:return 28;
    case 28:return 24;case 24:return 22;case 22:return 20;case 20:return 18;
    case 18:return 16;case 16:return 12;case 12:return 10;default:return 8;
    }
}
static void text(int cx,int y,const char *s,unsigned size,uint16_t color) {
    // Keep complete glyphs inside the circle, including near the top/bottom.
    int dy=abs(y-160)>abs(y+(int)size-160)?abs(y-160):abs(y+(int)size-160);
    int available=dy>=152?0:(int)(2*sqrtf(152*152-dy*dy))-2*abs(cx-160);
    if(available>260)available=260;
    if(available<=0)return;
    int width=hud_text_width(s,size);
    while(width>available && size>8) {
        size=smaller_font(size);
        width=hud_text_width(s,size);
    }
    if(width>available) {hud_text(cx-available/2,y,s,size,color,text_background,available);return;}
    hud_text(cx-width/2,y,s,size,color,text_background,available);
}
static int nav_strip_y;
static void paint_round_on(void (*draw)(void*),void *context,uint16_t background) {
    uint16_t previous_background=text_background;
    text_background=background;
    for(nav_strip_y=0;nav_strip_y<320;nav_strip_y+=8)
        tft_draw_buffered_region(0,nav_strip_y,320,8,background,draw,context);
    text_background=previous_background;
}
static void paint_round(void (*draw)(void*),void *context) {
    paint_round_on(draw,context,BLACK);
}
typedef struct {float x,y;} point_t;
typedef struct {point_t a,b;bool past;} path_segment_t;
// BLE route windows contain at most 16 points. Cache subdivision once per
// frame, not once per strip; keep allocations out of the navigation loop.
static path_segment_t path[15];
static unsigned path_count;
static const int RIDER_X=155,RIDER_Y=195;
static float maxf(float a,float b){return a>b?a:b;}
static int mini(int a,int b){return a<b?a:b;}
static int maxi(int a,int b){return a>b?a:b;}
static void stroke(point_t a,point_t b,float radius,uint16_t color,unsigned opacity) {
    int top=maxi(nav_strip_y,(int)(fminf(a.y,b.y)-radius-1));
    int bottom=mini(nav_strip_y+8,(int)(maxf(a.y,b.y)+radius+2));
    int left=maxi(0,(int)(fminf(a.x,b.x)-radius-1));
    int right=mini(320,(int)(maxf(a.x,b.x)+radius+2));
    float dx=b.x-a.x,dy=b.y-a.y,length=dx*dx+dy*dy;
    for(int y=top;y<bottom;y++)for(int x=left;x<right;x++) {
        float t=length?((x-a.x)*dx+(y-a.y)*dy)/length:0;
        if(t<0)t=0;
        if(t>1)t=1;
        float px=x-a.x-t*dx,py=y-a.y-t*dy,d2=px*px+py*py;
        if(d2>(radius+1)*(radius+1))continue;
        float coverage=radius+0.5f-sqrtf(d2);
        if(coverage>1)coverage=1;
        if(coverage>0)tft_blend_pixel(x,y,color,(unsigned)(coverage*opacity+0.5f));
    }
}
static point_t route_point(const round_nav_t *v,unsigned i) {
    if(v->overview)return (point_t){24+v->points[i].x*0.85f,48+v->points[i].y*0.72f};
    float z=v->zoom_percent/100.0f*1.25f;
    return (point_t){RIDER_X+((int)v->points[i].x-160)*z,
                     RIDER_Y+((int)v->points[i].y-160)*z};
}
static void prepare_route(const round_nav_t *v) {
    path_count=0;
    unsigned count=v->count>16?16:v->count;
    if(!v->points || count<2)return;
    for(unsigned i=1;i<count;i++)
        path[path_count++]=(path_segment_t){route_point(v,i-1),route_point(v,i),i<=v->completed};
}
static uint16_t shade(uint16_t c,float amount);
static void arrow(int cx,int cy,float angle,uint16_t color) {
    float a=angle*0.01745329252f;
    const point_t p[]={{0,-25},{-2,-24},{-20,12},{-21,16},{-19,18},
        {-16,18},{-1,9},{1,9},{16,18},{19,18},{21,16},{20,12},{2,-24}};
    unsigned count=sizeof(p)/sizeof(p[0]);
    float cs=cosf(a),sn=sinf(a);
    // Rasterize the rounded silhouette and bevel over the actual map. This
    // avoids the sharp flat triangle and preserves the reference's tail notch.
    for(int y=maxi(nav_strip_y,cy-36);y<mini(nav_strip_y+8,cy+36);y++)
    for(int x=maxi(0,cx-36);x<mini(320,cx+36);x++) {
        float px=((x-cx)*cs+(y-cy)*sn)/1.18f,py=(-(x-cx)*sn+(y-cy)*cs);
        bool inside=false;float nearest=10000;
        for(unsigned i=0,j=count-1;i<count;j=i++) {
            if((p[i].y>py)!=(p[j].y>py) &&
               px<(p[j].x-p[i].x)*(py-p[i].y)/(p[j].y-p[i].y)+p[i].x)inside=!inside;
            float dx=p[j].x-p[i].x,dy=p[j].y-p[i].y;
            float t=((px-p[i].x)*dx+(py-p[i].y)*dy)/(dx*dx+dy*dy);
            t=maxf(0,fminf(1,t));
            float xx=px-p[i].x-t*dx,yy=py-p[i].y-t*dy;
            nearest=fminf(nearest,xx*xx+yy*yy);
        }
        if(!inside && nearest>9)continue;
        float d=sqrtf(nearest);
        if(!inside) {
            if(d>1.4f)tft_blend_pixel(x,y,JR_RGB565(0x030705),(unsigned)fminf(15,(3-d)*10));
            else tft_blend_pixel(x,y,JR_RGB565(0x29322F),15);
        } else {
            float value=px<0?1.0f:0.95f;
            if(py>6)value=0.91f-(py-6)*0.012f;
            if(d<0.8f)value=0.80f;
            tft_blend_pixel(x,y,shade(color,value),15);
        }
    }
}
static void overlay_text(int x,int y,const char *s,unsigned size,uint16_t color,int width) {
    hud_text_overlay(x+1,y+1,s,size,BLACK,width);
    hud_text_overlay(x,y,s,size,color,width);
}
static uint16_t shade(uint16_t c,float amount) {
    if(amount>1)amount=1;
    if(amount<0)amount=0;
    return ((unsigned)(((c>>11)&31)*amount)<<11)|
        ((unsigned)(((c>>5)&63)*amount)<<5)|(unsigned)((c&31)*amount);
}
static void disk(int cx,int cy,float radius,uint16_t color) {
    int top=maxi(nav_strip_y,cy-radius-1),bottom=mini(nav_strip_y+8,cy+radius+2);
    for(int y=top;y<bottom;y++)for(int x=maxi(0,cx-radius-1);x<mini(320,cx+radius+2);x++) {
        float d=sqrtf((float)((x-cx)*(x-cx)+(y-cy)*(y-cy)));
        float coverage=radius+0.5f-d;if(coverage>1)coverage=1;
        if(coverage>0)tft_blend_pixel(x,y,color,(unsigned)(coverage*15+0.5f));
    }
}
static void turn_arrow(uint8_t direction) {
    if(direction==NAV_DIR_UTURN) {
        // Upright stem, rounded return and downward arrowhead, as referenced.
        stroke((point_t){70,268},(point_t){70,247},4.2f,WHITE,15);
        point_t previous={70,247};
        for(unsigned i=1;i<=16;i++) {
            float angle=3.14159265f+i*(3.14159265f/16);
            point_t next={82+12*cosf(angle),247+12*sinf(angle)};
            stroke(previous,next,4.2f,WHITE,15);
            previous=next;
        }
        stroke((point_t){94,247},(point_t){94,260},4.2f,WHITE,15);
        tft_fill_triangle(83,258,105,258,94,273,WHITE);
        return;
    }
    if(direction!=4 && direction!=5) {
        tft_draw_turn_arrow(82,253,37,(nav_direction_t)direction,WHITE);return;
    }
    float sign=direction==5?1:-1;
    point_t a={79,272},b={79,253},c={83,245},d={99,245};
    a.x=87+(a.x-87)*sign;b.x=87+(b.x-87)*sign;
    c.x=87+(c.x-87)*sign;d.x=87+(d.x-87)*sign;
    stroke(a,b,4.7f,WHITE,15);stroke(b,c,4.7f,WHITE,15);stroke(c,d,4.7f,WHITE,15);
    point_t tip={87+(113-87)*sign,245};
    tft_fill_triangle(lroundf(tip.x),245,lroundf(d.x),235,lroundf(d.x),255,WHITE);
}
static const char *action(uint8_t d) {
    switch(d) {
    case 2:return "Bear left";case 3:return "Bear right";
    case 4:case 6:return "Turn left";case 5:case 7:return "Turn right";
    case 8:return "Make U-turn";case 9:return "Arrived";
    case 10:return "Roundabout";case 11:return "Take exit";
    case 12:return "Merge";case 13:return "Keep fork";default:return "Go straight";
    }
}
static void draw_nav(void *context) {
    const round_nav_t *v=context;
    const hud_model_t *m=&v->nav;
    bool live=v->recovery==RIDE_LIVE;
    // Preserve the actual phone-projected polyline: no invented streets.
    for(unsigned i=0;i<path_count;i++) {
        unsigned alpha=!live?3:path[i].past?6:15;
        stroke(path[i].a,path[i].b,8.0f,ROUTE_GREEN,alpha/5);
        stroke(path[i].a,path[i].b,5.8f,ROUTE_GREEN,alpha);
    }
    if(v->count<2)text(160,125,"WAITING FOR ROUTE",12,MUTED);
    int rider_x=RIDER_X,rider_y=RIDER_Y;
    if(v->overview && v->count){point_t p=route_point(v,0);rider_x=lroundf(p.x);rider_y=lroundf(p.y);}
    // Local motion remains visible without a GPS fix. Grey still marks stale
    // guidance; the arrow never invents movement along the route.
    arrow(rider_x,rider_y,v->tilt,live?WHITE:MUTED);
    char value[24];
    // Clear just the lower guidance area after painting the route.
    tft_fill_rect(35,230,250,80,BLACK);
    if(live)turn_arrow(m->direction);
    const char *unit=m->distance_m>=1000?"km":"m";
    if(!live)strcpy(value,"--");
    else if(m->distance_m>=1000)snprintf(value,sizeof(value),"%.1f",m->distance_m/1000.0f);
    else snprintf(value,sizeof(value),"%lu",(unsigned long)m->distance_m);
    unsigned size=38;
    if(hud_text_width(value,size)+hud_text_width(unit,28)+8>170)size=28;
    if(hud_text_width(value,size)+hud_text_width(unit,28)+8>170)size=22;
    overlay_text(119,234,value,size,WHITE,170);
    unsigned unit_size=size<28?16:28;
    overlay_text(119+hud_text_width(value,size)+6,234+(size-unit_size),unit,unit_size,WHITE,45);
    const char *label=m->instruction;
    if(strcmp(label,"PHONE OFFLINE") && strcmp(label,"SYNCING") &&
       strcmp(label,"GPS LOST") && strcmp(label,"REROUTING") && strcmp(label,"PAUSED"))
        label=action(m->direction);
    if(!live)label="Guidance unavailable";
    unsigned label_size=hud_text_width(label,20)<=185?20:16;
    overlay_text(119,273,label,label_size,MUTED,185);
    if(live && v->trip_valid)snprintf(value,sizeof(value),"ETA %s  %.1f km",v->arrival_time[0]?v->arrival_time:"--:--",m->remaining_km);
    else strcpy(value,"ETA --:--   -- km");
    text(160,290,value,10,MUTED);
    static const char *states[]={"","RECONNECTING","SYNCING ROUTE","GPS WEAK","OFF ROUTE","PAUSED","WAITING FOR PHONE"};
    if(!live) {
        text(160,24,states[v->recovery<=RIDE_STALE?v->recovery:RIDE_STALE],12,GREEN);
        const char *status=v->recovery==RIDE_REROUTING
            ? (strstr(m->instruction,"retrying")?"RETRYING ROUTE":"RECALCULATING ROUTE")
            : "LAST ROUTE - DATA STALE";
        text(160,100,status,10,MUTED);
    } else if(v->battery_low) {
        snprintf(value,sizeof(value),"LOW BATTERY ~%u%%",v->battery_percent);
        text(160,24,value,10,GREEN);
    } else if(v->overview && v->destination[0]) {
        text(160,24,v->destination,10,GREEN);
    }
    if(ride_sharp_warning(m->direction,v->maneuver_state,m->distance_m,m->speed,v->recovery))
        text(160,108,m->direction==6?"! SHARP LEFT":"! SHARP RIGHT",12,GREEN);
    else if(live && v->overview && v->next_direction) {
        snprintf(value,sizeof(value),"Then %s",action(v->next_direction));
        text(160,108,value,10,MUTED);
    }
    // This is the rider's live GPS speed, labelled explicitly in km/h.
    bool speed_valid=live && m->speed_valid;
    disk(264,244,26,speed_valid?JR_RGB565(0xFF2020):MUTED);disk(264,244,21.5f,WHITE);
    if(speed_valid)snprintf(value,sizeof(value),"%u",m->speed);
    else strcpy(value,"--");
    unsigned speed_size=32;
    if(hud_text_width(value,speed_size)>36)speed_size=22;
    if(hud_text_width(value,speed_size)>36)speed_size=16;
    if(hud_text_width(value,speed_size)>36)speed_size=12;
    hud_text_overlay(264-hud_text_width(value,speed_size)/2,223+(32-speed_size)/2,value,speed_size,BLACK,45);
    hud_text_overlay(264-hud_text_width("km/h",8)/2,254,"km/h",8,BLACK,30);

}
static void draw_splash(void *unused) {
    (void)unused;
    tft_fill_round_rect(100,219,120,3,1,JR_PANEL_ALT);
}
void round_ui_splash_progress(unsigned progress) {
    if(progress>100)progress=100;
    tft_fill_round_rect(100,219,120,3,1,JR_PANEL_ALT);
    if(progress)tft_fill_round_rect(100,219,120*progress/100,3,1,JR_PRIMARY);
}
void round_ui_splash(unsigned progress) {
    paint_round(draw_splash,NULL);
#if CONFIG_JUSTRIDE_ROUND_AMOLED && !defined(HUD_CAPTURE)
    tft_spi_lock();
    round_panel_blit_rle((466-ROUND_SPLASH_LOGO_WIDTH)/2,178,
        ROUND_SPLASH_LOGO_WIDTH,ROUND_SPLASH_LOGO_HEIGHT,round_splash_logo,
        sizeof(round_splash_logo)/sizeof(round_splash_logo[0])/2);
    round_panel_flush();
    tft_spi_unlock();
#endif
    round_ui_splash_progress(progress);
}
void round_ui_navigation(const round_nav_t *v) {
    prepare_route(v);
    for(nav_strip_y=0;nav_strip_y<320;nav_strip_y+=8)
        tft_draw_buffered_region(0,nav_strip_y,320,8,BLACK,draw_nav,(void*)v);
}
static void draw_arrival(void *unused) {
    (void)unused;
    text(160,57,"DESTINATION REACHED",12,GREEN);
    tft_fill_circle(160,145,53,GREEN);tft_fill_circle(160,145,49,BLACK);
    tft_draw_thick_line(132,144,152,164,7,GREEN);
    tft_draw_thick_line(152,164,190,123,7,GREEN);
    text(160,222,"You've arrived",24,WHITE);
    text(160,259,"RIDE COMPLETE",12,GREEN);
    text(160,282,"Returning home shortly",10,MUTED);
}
void round_ui_arrival(void) {
    paint_round(draw_arrival,NULL);
}
typedef struct {uint32_t distance,duration;uint16_t speed;bool valid;} ride_summary_t;
static void draw_ride_summary(void *context) {
    const ride_summary_t *ride=context;
    char value[24];
    if(!ride->valid)return;
    unsigned width;
    int x;
    snprintf(value,sizeof(value),ride->distance?"%.1f km":"%.0f km",ride->distance/1000.0f);
    width=hud_text_width(value,32);x=(320-(36+(int)width))/2;
    tft_draw_circle(x+12,95,13,GREEN);
    tft_draw_thick_line(x+7,99,x+12,91,2,WHITE);
    tft_draw_thick_line(x+12,91,x+17,99,2,WHITE);
    tft_fill_circle(x+7,99,1.5f,WHITE);tft_fill_circle(x+17,99,1.5f,WHITE);
    text(178,78,value,32,WHITE);
    uint32_t hours=ride->duration/3600,minutes=(ride->duration%3600)/60;
    if(hours)snprintf(value,sizeof(value),"%lu hr %02lu min",(unsigned long)hours,(unsigned long)minutes);
    else snprintf(value,sizeof(value),"%lu min",(unsigned long)minutes);
    width=hud_text_width(value,28);x=(320-(36+(int)width))/2;
    tft_draw_circle(x+12,157,13,GREEN);
    tft_draw_circle(x+12,157,7,WHITE);
    tft_draw_thick_line(x+12,157,x+12,151,2,WHITE);
    tft_draw_thick_line(x+12,157,x+17,157,2,WHITE);
    hud_text(x+36,143,value,28,WHITE,BLACK,width);
    snprintf(value,sizeof(value),"%.1f km/h",ride->speed/10.0f);
    width=hud_text_width(value,28);x=(320-(36+(int)width))/2;
    tft_draw_circle(x+12,219,13,GREEN);
    tft_draw_thick_line(x+5,222,x+7,216,2,WHITE);
    tft_draw_thick_line(x+7,216,x+12,213,2,WHITE);
    tft_draw_thick_line(x+12,213,x+17,216,2,WHITE);
    tft_draw_thick_line(x+17,216,x+19,222,2,WHITE);
    tft_draw_thick_line(x+12,219,x+17,215,2,WHITE);
    tft_draw_thick_line(x+5,223,x+19,223,2,WHITE);
    hud_text(x+36,205,value,28,WHITE,BLACK,width);
}
void round_ui_ride_summary(uint32_t distance_m,uint32_t duration_s,uint16_t average_speed_tenths,bool valid) {
    ride_summary_t ride={distance_m,duration_s,average_speed_tenths,valid};
    paint_round(draw_ride_summary,&ride);
}
typedef struct {const char *clock,*date;uint32_t distance;uint8_t battery;bool valid;} home_t;
static void draw_home(void *context) {
    const home_t *h=context;
    char value[32];
    tft_draw_round_rect(143,38,34,22,5,WHITE);
    tft_fill_rect(178,44,3,10,WHITE);
    if(h->battery<=100) {
        uint16_t color=h->battery<=15?JR_RGB565(0xF04030):GREEN;
        unsigned battery_width=h->battery*26/100;
        if(battery_width)tft_fill_round_rect(147,42,battery_width,14,3,color);
    }
    text(160,91,h->clock,52,WHITE);
    tft_draw_thick_line(117,168,203,168,2,SURFACE_BORDER);
    tft_draw_thick_line(143,168,177,168,2,GREEN);
    if(!h->valid || h->distance==0)strcpy(value,"0 km");
    else snprintf(value,sizeof(value),"%.1f km",h->distance/1000.0f);
    unsigned distance_size=38;
    int distance_width=hud_text_width(value,distance_size);
    while(distance_width>218 && distance_size>22) {
        distance_size=smaller_font(distance_size);
        distance_width=hud_text_width(value,distance_size);
    }
    text(160,194,value,distance_size,GREEN);
    tft_draw_thick_line(146,263,151,257,2,MUTED);
    tft_draw_thick_line(151,257,158,266,2,MUTED);
    tft_draw_thick_line(158,266,165,257,2,MUTED);
    tft_draw_thick_line(165,257,174,263,2,MUTED);
}
void round_ui_home(const char *clock,const char *date,uint32_t distance_m,uint8_t battery_percent,bool valid) {
    home_t home={clock,date,distance_m,battery_percent,valid};paint_round(draw_home,&home);
}

typedef struct {const char *title,*artist,*status;bool playing,active;uint8_t actions;uint32_t elapsed,duration;} music_t;
static void skip(int cx,int cy,bool next,uint16_t color) {
    int sign=next?1:-1;
    tft_fill_triangle(cx-sign*11,cy-12,cx+sign*8,cy,cx-sign*11,cy+12,color);
    tft_fill_round_rect(cx+sign*12-2,cy-13,4,26,2,color);
}
static bool glove_mode;
void round_ui_set_glove_mode(bool enabled){glove_mode=enabled;}
static void draw_music(void *context) {
    const music_t *m=context;
    text(160,35,"NOW PLAYING",20,GREEN);
    if(!m->active) {
        tft_draw_circle(160,87,24,SURFACE_BORDER);
        tft_draw_thick_line(164,72,164,96,3,GREEN);
        tft_draw_thick_line(164,72,176,76,3,GREEN);
        tft_fill_circle(158,97,6,GREEN);
        text(160,128,"Phone music",28,WHITE);
        text(160,169,"Open music on your phone",16,MUTED);
        tft_draw_round_rect(60,205,200,48,24,GREEN);
        text(160,218,"Connect",22,WHITE);
        return;
    }
    text(160,78,m->title[0]?m->title:m->playing?"Music playing":"Music paused",32,WHITE);
    text(160,121,m->artist,20,MUTED);
    tft_fill_round_rect(32,173,256,6,3,JR_RGB565(0x104B3A));
    bool timed=m->active && m->duration>0;
    uint32_t elapsed=m->elapsed>m->duration?m->duration:m->elapsed;
    if(timed && elapsed)tft_fill_round_rect(32,173,(uint64_t)elapsed*256/m->duration,6,3,GREEN);
    char value[20];
    if(timed)snprintf(value,sizeof(value),"%lu:%02lu",(unsigned long)(elapsed/60),(unsigned long)(elapsed%60));
    else strcpy(value,"--:--");
    hud_text(32,184,value,18,MUTED,BLACK,100);
    if(timed)snprintf(value,sizeof(value),"%lu:%02lu",(unsigned long)(m->duration/60),(unsigned long)(m->duration%60));
    else strcpy(value,"--:--");
    hud_text(288-hud_text_width(value,18),184,value,18,MUTED,BLACK,100);
    if(m->status[0])text(160,151,m->status,10,MUTED);
    uint16_t previous=m->active&&(m->actions&8)?WHITE:JR_BORDER;
    uint16_t next=m->active&&(m->actions&4)?WHITE:JR_BORDER;
    uint16_t play=m->active&&(m->actions&(m->playing?2:1))?GREEN:JR_BORDER;
    skip(80,245,false,previous);skip(240,245,true,next);
    disk(160,245,glove_mode?41:39,play);
    if(m->playing){tft_fill_round_rect(148,230,7,30,3,BLACK);tft_fill_round_rect(166,230,7,30,3,BLACK);}
    else tft_fill_triangle(153,230,153,260,175,245,BLACK);
    text(118,278,"-",glove_mode?22:16,m->active&&(m->actions&16)?WHITE:JR_BORDER);
    text(202,278,"+",glove_mode?22:16,m->active&&(m->actions&16)?WHITE:JR_BORDER);
}
void round_ui_music(const char *title,const char *artist,const char *status,bool playing,uint8_t actions,bool active,uint32_t elapsed,uint32_t duration) {
    music_t m={title,artist,status,playing,active,actions,elapsed,duration};
    paint_round(draw_music,&m);
}

typedef struct {const char *caller,*number;bool connected;uint8_t actions;} call_t;
static void handset(int cx,int cy,bool accept) {
    const uint8_t *mask=accept?round_call_icon:round_call_end_icon;
    for(int y=maxi(0,nav_strip_y-cy+14);y<mini(28,nav_strip_y+8-cy+14);y++)
    for(int x=0;x<28;x++) {
        unsigned index=y*28+x;
        uint8_t pair=mask[index/2];
        unsigned alpha=index&1?pair&15:pair>>4;
        if(alpha)tft_blend_pixel(cx-14+x,cy-14+y,WHITE,alpha);
    }
}
static void draw_call(void *context) {
    const call_t *c=context;
    text(160,67,"Incoming Call",20,MUTED);
    text(160,104,c->caller[0]?c->caller:"Unknown caller",38,WHITE);
    const char *detail=!c->connected?"Phone disconnected":strcmp(c->caller,c->number)?c->number:"";
    text(160,150,detail,20,MUTED);
    uint16_t decline=c->connected&&(c->actions&2)?JR_RGB565(0xE9231C):JR_BORDER;
    uint16_t accept=c->connected&&(c->actions&1)?JR_RGB565(0x00B84A):JR_BORDER;
    int radius=glove_mode?35:26;
    disk(108,229,radius+3,shade(decline,.45f));disk(108,229,radius+1.5f,BLACK);disk(108,229,radius,decline);
    disk(208,229,radius+3,shade(accept,.45f));disk(208,229,radius+1.5f,BLACK);disk(208,229,radius,accept);
    handset(108,229,false);handset(208,229,true);
}
void round_ui_call(const char *caller,const char *number,uint8_t actions,bool connected) {
    call_t c={caller,number,connected,actions};paint_round(draw_call,&c);
}
void round_ui_notification(const char *app,const char *sender,const char *message) {
    tft_fill_round_rect(55,35,210,59,10,BLACK);tft_draw_round_rect(55,35,210,59,10,GREEN);
    hud_text(66,42,app,10,GREEN,BLACK,188);hud_text(66,56,sender,12,WHITE,BLACK,188);
    hud_text(66,74,message,10,MUTED,BLACK,188);
}

static void button(int x,int y,int w,int h,const char *label) {
    tft_fill_round_rect(x,y,w,h,h/2,SURFACE);
    tft_draw_round_rect(x,y,w,h,h/2,SURFACE_BORDER);
    text(x+w/2,y+(h-18)/2,label,18,WHITE);
}
typedef struct {bool connected;unsigned brightness;} settings_t;
static void draw_settings(void *context) {
    const settings_t *settings=context;
    const char *labels[]={"Bluetooth","Brightness"};
    char percent[8];snprintf(percent,sizeof(percent),"%u%%",settings->brightness);
    const char *values[]={settings->connected?"Connected":"Offline",percent};
    for(int i=0;i<2;i++) {
        int y=ROUND_SETTINGS_Y+i*ROUND_SETTINGS_STEP;
        tft_draw_fast_h_line(ROUND_SETTINGS_X+15,y+ROUND_ROW_H,ROUND_SETTINGS_W-30,SURFACE_BORDER);
        tft_fill_circle(ROUND_SETTINGS_X+16,y+17,3,
            i==0?(settings->connected?GREEN:MUTED):GREEN);
        hud_text(ROUND_SETTINGS_X+30,y+4,labels[i],16,MUTED,BLACK,180);
        unsigned value_size=22;
        int value_width=hud_text_width(values[i],value_size);
        hud_text(ROUND_SETTINGS_X+30,y+24,
            values[i],value_size,i==1?GREEN:settings->connected?GREEN:WHITE,
            BLACK,value_width);
    }
    tft_draw_round_rect(44,ROUND_ACTION_Y,110,ROUND_ACTION_H,23,SURFACE_BORDER);
    tft_draw_round_rect(166,ROUND_ACTION_Y,110,ROUND_ACTION_H,23,SURFACE_BORDER);
    text(99,ROUND_ACTION_Y+12,"Restart",20,WHITE);
    text(221,ROUND_ACTION_Y+12,"Reset",20,WHITE);
}
void round_ui_settings(bool connected,unsigned brightness) {
    settings_t settings={connected,brightness};
    paint_round(draw_settings,&settings);
}
static void draw_brightness(void *context) {
    unsigned brightness=*(unsigned*)context;
    text(160,49,"Brightness",24,GREEN);
    char value[8];snprintf(value,sizeof(value),"%u%%",brightness);
    text(160,103,value,52,WHITE);
    tft_fill_round_rect(ROUND_SLIDER_X,ROUND_SLIDER_Y,ROUND_SLIDER_WIDTH,8,4,JR_BORDER);
    if(brightness)tft_fill_round_rect(ROUND_SLIDER_X,ROUND_SLIDER_Y,
        brightness*ROUND_SLIDER_WIDTH/100,8,4,GREEN);
    tft_fill_circle(ROUND_SLIDER_X+brightness*ROUND_SLIDER_WIDTH/100,
        ROUND_SLIDER_Y+4,12,GREEN);
}
void round_ui_brightness(unsigned brightness) {
    paint_round(draw_brightness,&brightness);
}
void round_ui_update(unsigned progress) {
    tft_fill_screen(BLACK);
    text(160,62,"FIRMWARE UPDATE",20,GREEN);
    char value[8];snprintf(value,sizeof(value),"%u%%",progress);
    text(160,112,value,38,WHITE);
    tft_draw_round_rect(70,180,180,16,6,JR_BORDER);
    if(progress)tft_fill_round_rect(73,183,(int)(progress*174/100),10,4,GREEN);
    text(160,221,progress==100?"VERIFYING FIRMWARE":"RECEIVING UPDATE",16,WHITE);
    text(160,247,"KEEP DEVICE POWERED ON",10,MUTED);
}
static void draw_reset(void *unused) {
    (void)unused;
    text(160,72,"Reset device?",24,GREEN);
    text(160,123,"Restore settings and",18,WHITE);
    text(160,149,"clear Bluetooth pairing?",18,WHITE);
    button(52,202,100,40,"Cancel");
    button(168,202,100,40,"Reset");
}
void round_ui_reset(void) {
    paint_round(draw_reset,NULL);
}
void round_ui_pairing(uint32_t passkey) {
    tft_fill_screen(BLACK);
    text(160,77,"PAIR YOUR PHONE",20,GREEN);
    char code[7];snprintf(code,sizeof(code),"%06lu",(unsigned long)(passkey%1000000u));
    text(160,126,code,38,WHITE);
    text(160,195,"Enter this code on your phone",12,MUTED);
}
