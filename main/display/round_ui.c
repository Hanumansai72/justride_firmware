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
#define GREEN JR_RGB565(0x00F0A0)
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
static path_segment_t path[15*8];
static unsigned path_count;
static float map_scale;
static const int RIDER_X=155,RIDER_Y=211;
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
    float z=v->zoom_percent/100.0f;
    return (point_t){RIDER_X+((int)v->points[i].x-160)*z,
                     RIDER_Y+((int)v->points[i].y-160)*z};
}
static void prepare_route(const round_nav_t *v) {
    path_count=0;
    unsigned count=v->count>16?16:v->count;
    if(!v->points || count<2)return;
    for(unsigned i=1;i<count;i++) {
        point_t p0=route_point(v,i>1?i-2:0),p1=route_point(v,i-1);
        point_t p2=route_point(v,i),p3=route_point(v,i+1<count?i+1:i);
        point_t previous=p1;
        // Hermite corner rounding, bounded to the two adjacent waypoints.
        // Subdivision improves the stroke, not the phone's navigation data.
        for(unsigned s=1;s<=8;s++) {
            float t=s/8.0f,t2=t*t,t3=t2*t;
            float h0=2*t3-3*t2+1,h1=t3-2*t2+t,h2=-2*t3+3*t2,h3=t3-t2;
            point_t next={h0*p1.x+h1*(p2.x-p0.x)*0.5f+h2*p2.x+h3*(p3.x-p1.x)*0.5f,
                h0*p1.y+h1*(p2.y-p0.y)*0.5f+h2*p2.y+h3*(p3.y-p1.y)*0.5f};
            float minx=fminf(p1.x,p2.x),maxx=maxf(p1.x,p2.x);
            float miny=fminf(p1.y,p2.y),maxy=maxf(p1.y,p2.y);
            next.x=maxf(minx,fminf(maxx,next.x));next.y=maxf(miny,fminf(maxy,next.y));
            path[path_count++]=(path_segment_t){previous,next,i<=v->completed};
            previous=next;
        }
    }
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
        tft_draw_turn_arrow(82,258,37,(nav_direction_t)direction,WHITE);return;
    }
    float sign=direction==5?1:-1;
    point_t a={79,277},b={79,258},c={83,250},d={99,250};
    a.x=87+(a.x-87)*sign;b.x=87+(b.x-87)*sign;
    c.x=87+(c.x-87)*sign;d.x=87+(d.x-87)*sign;
    stroke(a,b,4.7f,WHITE,15);stroke(b,c,4.7f,WHITE,15);stroke(c,d,4.7f,WHITE,15);
    point_t tip={87+(113-87)*sign,250};
    tft_fill_triangle(lroundf(tip.x),250,lroundf(d.x),240,lroundf(d.x),260,WHITE);
}
static void reference_roads(void) {
    // Hand-placed junctions traced from the reference, not spaced line loops
    // or repeated tiles. Used ONLY by the explicitly marked initial demo.
    static const struct {int16_t ax,ay,bx,by;uint8_t width,level;} roads[]={
        {8,160,96,127,8,61},{96,127,72,99,8,61},{72,99,151,25,6,50},
        {8,229,88,183,12,66},{88,183,65,146,12,66},{65,146,164,106,10,66},
        {164,106,191,91,8,54},{191,91,269,43,6,41},
        {10,103,72,86,6,48},{72,86,100,54,5,44},{100,54,190,19,4,36},
        {83,39,138,56,4,41},{138,56,227,38,4,35},
        {96,127,113,114,6,47},{113,114,94,91,6,47},{94,91,125,71,5,42},
        {164,106,241,150,9,57},{241,150,309,116,9,57},
        {241,150,315,207,10,64},{214,135,271,103,5,39},
        {9,194,44,176,5,36},{44,176,25,148,5,36},
        {34,122,43,142,5,33},{102,140,123,168,5,37},
        {123,168,150,153,5,37},{150,153,174,177,5,35},
        {89,207,120,186,5,33},{120,186,140,212,5,33},
        {183,209,222,180,6,39},{222,180,240,198,6,39},
        {240,198,209,222,6,39},{209,222,185,200,6,39},
        {204,162,224,149,4,32},{260,176,283,161,5,35},
        {283,161,298,178,5,35},{75,236,105,213,6,32},
        {105,213,128,242,6,32},{256,224,280,205,6,32},
        {280,205,309,228,6,32},{140,30,168,43,3,28},
        {12,133,35,127,4,27},{35,127,43,142,4,27},
        {23,113,30,126,4,25},{13,154,30,147,4,26},
        {46,103,54,119,4,28},{54,119,77,110,4,28},
        {53,166,65,183,4,28},{65,183,37,199,4,28},
        {28,215,42,235,4,25},{42,235,63,222,4,25},
        {113,114,137,103,4,29},{112,84,133,94,4,27},
        {174,57,196,47,4,27},{196,47,220,57,4,27},
        {220,57,202,69,4,27},{202,69,183,61,4,27},
        {182,117,164,131,5,31},{164,131,145,120,5,31},
        {149,155,176,138,4,28},{176,138,192,150,4,28},
        {207,189,196,178,4,28},{196,178,217,163,4,28},
        {250,133,268,144,4,29},{268,144,295,131,4,29},
        {276,88,294,100,4,26},{294,100,317,89,4,26},
        {145,239,159,227,4,25},{159,227,177,243,4,25}
    };
    for(unsigned i=0;i<sizeof(roads)/sizeof(roads[0]);i++) {
        float y=(roads[i].ay+roads[i].by)*0.5f;
        float dx=((roads[i].ax+roads[i].bx)*0.5f-160)/160,dy=(y-160)/160;
        float dim=1-0.65f*(dx*dx+dy*dy);
        if(y>228)dim*=1.0f-(y-228)*0.0065f;
        unsigned gray=(unsigned)(roads[i].level*dim);
        uint16_t color=JR_RGB565((gray<<16)|(gray<<8)|gray);
        stroke((point_t){roads[i].ax,roads[i].ay},(point_t){roads[i].bx,roads[i].by},
               roads[i].width/2.0f,color,15);
    }
}
static void draw_nav(void *context) {
    const round_nav_t *v=context;
    if(v->demo)reference_roads();
    if(v->map && v->map_w && v->map_h) {
        // One aspect-preserving transform covers the complete circular dial.
        // It places the phone's live marker at the reference's lower center.
        uint8_t line[640];
        for(int y=nav_strip_y;y<nav_strip_y+8 && y<320;y++) {
            int sy=(int)(v->marker_y+(y-RIDER_Y)/map_scale);
            for(int x=0;x<320;x++) {
                int sx=(int)(v->marker_x+(x-RIDER_X)/map_scale);
                uint16_t color=BLACK;
                if(sx>=0 && sx<(int)v->map_w && sy>=0 && sy<(int)v->map_h-24) {
                    const uint8_t *p=v->map+(sy*v->map_w+sx)*2;color=(p[0]<<8)|p[1];
                }
                // Reference vignette: streets stay visible behind the lower
                // guidance; there is no rectangular black footer.
                float dx=(x-160)/160.0f,dy=(y-160)/160.0f;
                float dim=1.0f-0.48f*(dx*dx+dy*dy);
                if(y>228)dim*=1.0f-(y-228)*0.0065f;
                color=shade(color,dim);
                line[x*2]=color>>8;line[x*2+1]=color;
            }
            tft_draw_raw_pixels(0,y,320,1,line);
        }
    }
    if(!v->map_contains_route) {
        for(unsigned layer=0;layer<3;layer++)for(unsigned i=0;i<path_count;i++) {
            path_segment_t *s=&path[i];
            float width=2.8f+4.0f*fminf(maxf(s->a.y,s->b.y)/RIDER_Y,1);
            uint16_t color=s->past?MUTED:shade(GREEN,0.25f+0.75f*fminf(maxf(s->a.y,s->b.y)/45,1));
            stroke(s->a,s->b,width+(layer==0?6:layer==1?3:0),
                color,layer==0?1:layer==1?3:15);
        }
        if(!v->map && v->count<2)text(160,112,"WAITING FOR ROUTE",12,MUTED);
    }
    // Compass: thin gray rim, red north pointer, white N below it.
    disk(262,80,25,JR_RGB565(0x6D7375));disk(262,80,23,BLACK);
    float north=-(float)v->nav.heading*0.01745329252f;
    point_t compass[4]={{0,-11},{-6,6},{0,2},{6,6}};
    for(unsigned i=0;i<4;i++) {float x=compass[i].x;
        compass[i]=(point_t){262+x*cosf(north)-compass[i].y*sinf(north),
            72+x*sinf(north)+compass[i].y*cosf(north)};}
    tft_fill_triangle(lroundf(compass[0].x),lroundf(compass[0].y),lroundf(compass[1].x),lroundf(compass[1].y),
        lroundf(compass[2].x),lroundf(compass[2].y),0xF800);
    tft_fill_triangle(lroundf(compass[0].x),lroundf(compass[0].y),lroundf(compass[2].x),lroundf(compass[2].y),
        lroundf(compass[3].x),lroundf(compass[3].y),0xF800);
    overlay_text(262-hud_text_width("N",16)/2,80,"N",16,WHITE,30);
    arrow(RIDER_X,RIDER_Y,v->tilt);
    const hud_model_t *m=&v->nav;
    turn_arrow(m->direction);
    char distance[18];
    if(m->distance_m>=1000)snprintf(distance,sizeof(distance),"%.1f",m->distance_m/1000.0f);
    else snprintf(distance,sizeof(distance),"%lu",(unsigned long)m->distance_m);
    unsigned size=36;
    if(hud_text_width(distance,size)>98)size=28;
    if(hud_text_width(distance,size)>98)size=22;
    if(hud_text_width(distance,size)>98)size=16;
    overlay_text(119,238,distance,size,WHITE,110);
    overlay_text(123+hud_text_width(distance,size),255,m->distance_m>=1000?"km":"m",22,WHITE,35);
    const char *label=m->instruction;
    // Ordinary street names must not replace the turn action.
    if(strcmp(label,"PHONE OFFLINE") && strcmp(label,"SYNCING") && strcmp(label,"GPS LOST") && strcmp(label,"REROUTING")) {
        switch(m->direction){
            case 2:label="Bear left";break;case 3:label="Bear right";break;
            case 4:case 6:label="Turn left";break;case 5:case 7:label="Turn right";break;
            case 8:label="Make U-turn";break;case 9:label="Arrived";break;
            case 10:label="Roundabout";break;case 11:label="Take exit";break;
            default:label="Continue straight";break;
        }
    }
    overlay_text(119,279,label,16,MUTED,185);
    disk(261,244,28,JR_RGB565(0xFF2028));disk(261,244,22,WHITE);
    char speed[8];
    if(m->speed_limit)snprintf(speed,sizeof(speed),"%u",m->speed_limit);
    else strcpy(speed,"--");
    unsigned speed_size=strlen(speed)>2?22:28;
    hud_text_overlay(261-hud_text_width(speed,speed_size)/2,226,speed,speed_size,BLACK,52);
    if(v->map && v->map_contains_route)
        overlay_text(80,303,"(c) LocationIQ (c) OSM",8,MUTED,180);
    if(v->demo)overlay_text(146,309,"DEMO",8,MUTED,40);
}
void round_ui_navigation_demo(void) {
    static const round_route_point_t points[]={{160,160},{160,142},{160,129},
        {172,118},{191,106},{197,94},{192,84},{177,76},{156,68},{149,60},
        {155,51},{173,41},{175,33},{164,21},{163,12}};
    const round_nav_t v={.nav={.speed_limit=70,.direction=5,.distance_m=300},
        .points=points,.count=sizeof(points)/sizeof(points[0]),.zoom_percent=140,.demo=true};
    round_ui_navigation(&v);
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
    // Crop to cover around the live marker, rather than sampling negative rows
    // and leaving the top half of the display empty.
    unsigned usable_h=v->map_h>24?v->map_h-24:0;
    float mx=v->marker_x,my=v->marker_y;
    map_scale=1;
    if(v->map && mx>0 && my>0 && mx<v->map_w && my<usable_h) {
        map_scale=maxf(RIDER_X/mx,(320-RIDER_X)/(v->map_w-mx));
        map_scale=maxf(map_scale,maxf(RIDER_Y/my,(320-RIDER_Y)/(usable_h-my)));
    }
    for(nav_strip_y=0;nav_strip_y<320;nav_strip_y+=8)
        tft_draw_buffered_region(0,nav_strip_y,320,8,BLACK,draw_nav,(void*)v);
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
