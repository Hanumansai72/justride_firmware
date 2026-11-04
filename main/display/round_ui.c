#include "round_ui.h"
#ifdef HUD_CAPTURE
#include "hud_capture_driver.h"
#else
#include "display_tft.h"
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "splash_assets.h"

#define BLACK 0x0000
#define WHITE 0xFFFF
#define GREEN JR_PRIMARY
#define MUTED JR_RGB565(0xAEB6BC)
static void text(int cx,int y,const char *s,unsigned size,uint16_t color) {
    int width=hud_text_width(s,size);
    if(width>260) {size=16;width=hud_text_width(s,size);}
    if(width>260) {hud_text(30,y,s,size,color,BLACK,260);return;}
    hud_text(cx-width/2,y,s,size,color,BLACK,260);
}
static int nav_strip_y;
typedef struct {float x,y;} point_t;
typedef struct {point_t a,b;bool past;} path_segment_t;
// BLE route windows contain at most 16 points. Cache subdivision once per
// frame, not once per strip; keep allocations out of the navigation loop.
static path_segment_t path[15];
static unsigned path_count;
static const int RIDER_X=155,RIDER_Y=193;
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
    float z=v->zoom_percent/100.0f;
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
static void arrow(int cx,int cy,float angle) {
    float a=angle*0.01745329252f;
    const point_t p[]={{0,-25},{-2,-24},{-20,12},{-21,16},{-19,18},
        {-16,18},{-1,12},{1,12},{16,18},{19,18},{21,16},{20,12},{2,-24}};
    unsigned count=sizeof(p)/sizeof(p[0]);
    float cs=cosf(a),sn=sinf(a);
    // Rasterize the rounded silhouette and bevel over the actual map. This
    // avoids the sharp flat triangle and preserves the reference's tail notch.
    for(int y=maxi(nav_strip_y,cy-32);y<mini(nav_strip_y+8,cy+32);y++)
    for(int x=maxi(0,cx-32);x<mini(320,cx+32);x++) {
        float px=(x-cx)*cs+(y-cy)*sn,py=-(x-cx)*sn+(y-cy)*cs;
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
        if(!inside && nearest>25)continue;
        float d=sqrtf(nearest);
        if(!inside) {
            if(d>2)tft_blend_pixel(x,y,JR_RGB565(0x030705),(unsigned)fminf(15,(5-d)*7));
            else tft_blend_pixel(x,y,JR_RGB565(0x8B9691),15);
        } else {
            float value=py<6?0.98f:0.98f-(py-6)*0.009f;
            if(px<0)value-=0.035f;
            uint16_t color=shade(WHITE,value);
            tft_blend_pixel(x,y,color,15);
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
static void disk(int cx,int cy,int radius,uint16_t color) {
    int top=maxi(nav_strip_y,cy-radius-1),bottom=mini(nav_strip_y+8,cy+radius+2);
    for(int y=top;y<bottom;y++)for(int x=maxi(0,cx-radius-1);x<mini(320,cx+radius+2);x++) {
        float d=sqrtf((float)((x-cx)*(x-cx)+(y-cy)*(y-cy)));
        float coverage=radius+0.5f-d;if(coverage>1)coverage=1;
        if(coverage>0)tft_blend_pixel(x,y,color,(unsigned)(coverage*15+0.5f));
    }
}
static void turn_arrow(uint8_t direction) {
    if(direction!=4 && direction!=5) {
        tft_draw_turn_arrow(82,246,37,(nav_direction_t)direction,GREEN);return;
    }
    float sign=direction==5?1:-1;
    point_t a={79,265},b={79,246},c={83,238},d={99,238};
    a.x=87+(a.x-87)*sign;b.x=87+(b.x-87)*sign;
    c.x=87+(c.x-87)*sign;d.x=87+(d.x-87)*sign;
    stroke(a,b,4.7f,GREEN,15);stroke(b,c,4.7f,GREEN,15);stroke(c,d,4.7f,GREEN,15);
    point_t tip={87+(113-87)*sign,238};
    tft_fill_triangle(lroundf(tip.x),238,lroundf(d.x),228,lroundf(d.x),248,GREEN);
}
static void turn_chevrons(int side) {
    // Same six-point silhouette as the supplied reference, mirrored for right.
    for(int i=0;i<3;i++) {
        int x=96+i*36,y=116;
        int px[]={x+35,x+58,x+23,x+58,x+35,x};
        int py[]={y,y,y+42,y+84,y+84,y+42};
        if(side>0)for(int j=0;j<6;j++)px[j]=320-px[j];
        uint16_t c=i==1?GREEN:shade(GREEN,0.70f);
        tft_fill_triangle(px[0],py[0],px[1],py[1],px[2],py[2],c);
        tft_fill_triangle(px[0],py[0],px[2],py[2],px[5],py[5],c);
        tft_fill_triangle(px[5],py[5],px[2],py[2],px[3],py[3],c);
        tft_fill_triangle(px[5],py[5],px[3],py[3],px[4],py[4],c);
    }
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
    bool alert=ride_turn_alert(m->direction,v->maneuver_state,m->distance_m,v->recovery) && !v->overview;
    // Preserve the actual phone-projected polyline: no invented streets.
    for(unsigned i=0;i<path_count;i++)
        stroke(path[i].a,path[i].b,3.2f,GREEN,!live?3:path[i].past?6:15);
    if(v->count<2)text(160,125,"WAITING FOR ROUTE",12,MUTED);
    int rider_x=RIDER_X,rider_y=RIDER_Y;
    if(v->overview && v->count){point_t p=route_point(v,0);rider_x=lroundf(p.x);rider_y=lroundf(p.y);}
    if(live && !alert)arrow(rider_x,rider_y,v->overview?m->heading:v->tilt);
    if(alert) {
        tft_fill_rect(45,111,230,100,BLACK);
        if(v->blink_on)turn_chevrons(ride_turn_side(m->direction));
    }
    char value[24];
    if(m->speed_valid && live)snprintf(value,sizeof(value),"%u",m->speed);
    else strcpy(value,"--");
    overlay_text(54,52,value,28,WHITE,80);
    overlay_text(55,83,"km/h",10,MUTED,55);
    disk(262,80,22,MUTED);disk(262,80,20,BLACK);
    float north=-m->heading*0.01745329252f;
    tft_draw_thick_line(262,77,262+lroundf(11*sinf(north)),
        77-lroundf(11*cosf(north)),3,WHITE);
    overlay_text(257,85,"N",10,WHITE,20);
    // Clear just the lower guidance area after painting the route.
    tft_fill_rect(35,222,250,88,BLACK);
    if(live)turn_arrow(m->direction);
    if(!live)strcpy(value,"-- m");
    else if(m->distance_m>=1000)snprintf(value,sizeof(value),"%.1f km",m->distance_m/1000.0f);
    else snprintf(value,sizeof(value),"%lu m",(unsigned long)m->distance_m);
    unsigned size=hud_text_width(value,32)>170?22:32;
    overlay_text(119,226,value,size,WHITE,170);
    const char *label=m->instruction;
    if(strcmp(label,"PHONE OFFLINE") && strcmp(label,"SYNCING") &&
       strcmp(label,"GPS LOST") && strcmp(label,"REROUTING") && strcmp(label,"PAUSED"))
        label=action(m->direction);
    if(!live)label="Guidance unavailable";
    overlay_text(102,265,label,12,WHITE,185);
    if(live && v->trip_valid)snprintf(value,sizeof(value),"ETA %s  %.1f km",v->arrival_time[0]?v->arrival_time:"--:--",m->remaining_km);
    else strcpy(value,"ETA --:--   -- km");
    text(160,289,value,10,MUTED);
    static const char *states[]={"","RECONNECTING","SYNCING ROUTE","GPS WEAK","REROUTING","PAUSED","WAITING FOR PHONE"};
    if(!live) {
        text(160,24,states[v->recovery<=RIDE_STALE?v->recovery:RIDE_STALE],12,GREEN);
        text(160,100,"LAST ROUTE - DATA STALE",10,MUTED);
    } else if(v->battery_low) {
        snprintf(value,sizeof(value),"LOW BATTERY ~%u%%",v->battery_percent);
        text(160,24,value,10,GREEN);
    }
    if(m->speed_limit) {
        snprintf(value,sizeof(value),"LIMIT %u",m->speed_limit);
        overlay_text(185,37,value,10,MUTED,100);
    }
}
static void splash_bitmap(int x,int y,int w,int h,const uint8_t *bits,uint16_t color) {
    for(int py=0;py<h;py++)for(int px=0;px<w;px++)
        if(bits[py*((w+7)/8)+px/8]&(0x80>>(px%8)))tft_draw_pixel(x+px,y+py,color);
}
static void draw_splash(void *unused) {
    (void)unused;
    // Existing brand masks on the round canvas, with a plain black footer.
    int x=(320-SPLASH_LOGO_JUST_WIDTH-SPLASH_LOGO_RIDE_WIDTH)/2;
#if SPLASH_SHOW_ROUTE_ICON
    x-=17;
    splash_bitmap(x,109,24,24,splash_logo_icon_bits,JR_PRIMARY);x+=34;
#endif
    splash_bitmap(x,104,SPLASH_LOGO_JUST_WIDTH,SPLASH_LOGO_JUST_HEIGHT,splash_logo_just_bits,WHITE);
    splash_bitmap(x+SPLASH_LOGO_JUST_WIDTH,104,SPLASH_LOGO_RIDE_WIDTH,SPLASH_LOGO_RIDE_HEIGHT,splash_logo_ride_bits,JR_PRIMARY);
    text(160,152,"M A D E  F O R  B I K E R S",10,WHITE);
    tft_fill_round_rect(65,185,190,4,2,JR_PANEL_ALT);
    text(160,214,"G E A R  U P .  R I D E  O N .",8,MUTED);
}
void round_ui_splash_progress(unsigned progress) {
    if(progress>100)progress=100;
    tft_fill_round_rect(65,185,190,4,2,JR_PANEL_ALT);
    if(progress)tft_fill_round_rect(65,185,190*progress/100,4,2,JR_PRIMARY);
}
void round_ui_splash(unsigned progress) {
    tft_draw_buffered_region(0,0,320,320,BLACK,draw_splash,NULL);
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
    tft_draw_buffered_region(0,0,320,320,BLACK,draw_arrival,NULL);
}
typedef struct {uint8_t hours,minutes;const char *date;float remaining;} clock_t;
static void ring(float start,float end,uint16_t color) {
    for(float a=start;a<end;a+=1) {
        float r=0.01745329252f,b=fminf(a+1,end);
        tft_draw_thick_line(160+lroundf(151*cosf(a*r)),160+lroundf(151*sinf(a*r)),
            160+lroundf(151*cosf(b*r)),160+lroundf(151*sinf(b*r)),3,color);
    }
}
static void draw_clock(void *context) {
    const clock_t *c=context;
    ring(132,408,GREEN);
    ring(250,290,JR_PRIMARY_DARK);
    text(160,46,"JUSTRIDE",12,GREEN);
    char time[8];snprintf(time,sizeof(time),"%02u:%02u",c->hours,c->minutes);
    text(160,105,time,52,WHITE);text(160,170,c->date,12,MUTED);
    tft_fill_circle(160,224,17,JR_ACCENT_DIM);
    tft_fill_circle(160,220,8,GREEN);tft_fill_circle(160,220,3,BLACK);
    tft_fill_triangle(153,224,167,224,160,236,GREEN);
    char distance[24];if(c->remaining>0)snprintf(distance,sizeof(distance),"%.1f km",c->remaining);else strcpy(distance,"-- km");
    text(160,246,distance,28,WHITE);text(160,282,"ROUTE REMAINING",8,MUTED);
}
void round_ui_clock(uint8_t hours,uint8_t minutes,const char *date,float remaining_km) {
    clock_t c={hours,minutes,date,remaining_km};
    tft_draw_buffered_region(0,0,320,320,BLACK,draw_clock,&c);
}
typedef struct {const char *title,*artist,*status;bool playing,active;uint8_t actions;} music_t;
static void skip(int cx,int cy,bool next,uint16_t color) {
    int s=next?1:-1;tft_fill_triangle(cx-s*10,cy-11,cx+s*7,cy,cx-s*10,cy+11,color);
    tft_fill_rect(cx+s*10-2,cy-12,4,24,color);
}
static void draw_music(void *context) {
    const music_t *m=context;
    text(160,43,"NOW PLAYING",16,GREEN);
    text(160,87,m->title[0]?m->title:"No track selected",28,WHITE);
    text(160,130,m->artist,22,MUTED);
    // BLE has no elapsed/duration fields; do not draw a fictitious progress bar.
    text(160,170,m->status,10,MUTED);
    uint16_t previous=m->active&&(m->actions&8)?WHITE:JR_BORDER;
    uint16_t next=m->active&&(m->actions&4)?WHITE:JR_BORDER;
    uint16_t play=m->active&&(m->actions&(m->playing?2:1))?GREEN:JR_BORDER;
    skip(80,226,false,previous);skip(240,226,true,next);
    tft_fill_circle(160,226,33,play);
    if(m->playing){tft_fill_round_rect(148,214,7,25,3,BLACK);tft_fill_round_rect(166,214,7,25,3,BLACK);}
    else tft_fill_triangle(152,211,152,241,174,226,BLACK);
    text(118,278,"-",16,m->active&&(m->actions&16)?WHITE:JR_BORDER);
    text(202,278,"+",16,m->active&&(m->actions&16)?WHITE:JR_BORDER);
}
void round_ui_music(const char *title,const char *artist,const char *status,bool playing,uint8_t actions,bool active) {
    music_t m={title,artist,status,playing,active,actions};
    tft_draw_buffered_region(0,0,320,320,BLACK,draw_music,&m);
}
typedef struct {const char *caller;uint8_t actions;bool connected;} call_t;
static void draw_call(void *context) {
    const call_t *c=context;text(160,72,"Incoming Call",16,MUTED);
    text(160,116,c->caller[0]?c->caller:"Unknown caller",36,WHITE);
    text(160,165,c->connected?"Phone call":"Phone disconnected",12,MUTED);
    uint16_t decline=c->connected&&(c->actions&2)?0xE800:JR_BORDER;
    uint16_t accept=c->connected&&(c->actions&1)?0x05E0:JR_BORDER;
    tft_fill_circle(111,235,27,decline);tft_fill_circle(209,235,27,accept);
    // Horizontal handset for decline; slanted handset for accept.
    tft_draw_arc(111,240,15,10,210,330,WHITE);
    tft_fill_round_rect(97,231,6,9,2,WHITE);tft_fill_round_rect(119,231,6,9,2,WHITE);
    tft_draw_thick_line(200,224,205,237,5,WHITE);tft_draw_thick_line(205,237,218,243,5,WHITE);
    tft_fill_round_rect(196,220,8,7,2,WHITE);tft_fill_round_rect(215,237,7,8,2,WHITE);
}
void round_ui_call(const char *caller,uint8_t actions,bool connected) {
    call_t c={caller,actions,connected};tft_draw_buffered_region(0,0,320,320,BLACK,draw_call,&c);
}
void round_ui_notification(const char *app,const char *sender,const char *message) {
    tft_fill_round_rect(55,35,210,59,10,BLACK);tft_draw_round_rect(55,35,210,59,10,GREEN);
    hud_text(66,42,app,10,GREEN,BLACK,188);hud_text(66,56,sender,12,WHITE,BLACK,188);
    hud_text(66,74,message,10,MUTED,BLACK,188);
}
