// the brain screen: a plain text auton selector
// black background, gold accents, Montserrat font, a small gold bat in the top-right corner
// raw LVGL (not LLEMU) so we control the colors and font
// everything touching an LVGL object runs inside LVGL's own task: the button handlers (LVGL calls
// them itself) and the ui timer (an lv_timer runs inside LVGL too)

#include "main.h"
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include <algorithm>      // std::sort
#include <cmath>
#include <cstdio>
#include <iterator> // std::size

namespace {

constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240;

constexpr uint32_t BLACK = 0x000000;
constexpr uint32_t GOLD = 0xE8B21A; // accent for the bat, borders and headings
constexpr uint32_t WHITE = 0xFFFFFF;
constexpr uint32_t GRAY = 0x9A9AA0;
constexpr uint32_t BTN_BG = 0x161616;

// ---- bat shape, just for the little corner symbol ----
// only the right half is written down, buildBat() mirrors it for the left half
constexpr lv_point_t BAT_HALF[] = {
    {240, 171}, {251, 152}, {262, 142}, {271, 139}, {279, 144},
    {291, 133}, {303, 128}, {315, 127}, {327, 132}, {343, 123},
    {359, 115}, {373, 107}, {387, 99},  {359, 87},  {332, 82},
    {307, 83},  {282, 90},  {273, 66},  {258, 93},  {249, 93}, {240, 105},
};
constexpr int HALF_COUNT = std::size(BAT_HALF);
constexpr int BAT_COUNT = HALF_COUNT * 2 - 2; // the two end points already sit on the mirror line
constexpr float BAT_CX = 240.0f;              // centre of the stored shape
constexpr float BAT_CY = 118.0f;

// where the corner bat sits and how small it is
constexpr float CORNER_SCALE = 0.32f;
constexpr int CORNER_CX = 424;
constexpr int CORNER_CY = 34;

lv_obj_t* selectorObj = nullptr;
lv_timer_t* uiTimer = nullptr;
lv_obj_t* autonLabel = nullptr;
lv_obj_t* slotLabel = nullptr;
lv_obj_t* lockBtnLabel = nullptr;

int autonIndex = 0;       // which slot of AUTON_ROUTES is picked
bool autonLocked = false; // true once the driver locked the choice in

// ---- drawing the little bat ----
// LVGL has no "fill this outline" call, so fill it a row of pixels at a time: per row find where
// the edges cross it, sort them, fill between pairs

void drawSpan(lv_draw_ctx_t* draw_ctx, lv_draw_rect_dsc_t* dsc, int x1, int x2, int y) {
    if (x2 < x1) return;
    lv_area_t area;
    area.x1 = x1;
    area.y1 = y;
    area.x2 = x2;
    area.y2 = y; // one pixel tall
    lv_draw_rect(draw_ctx, dsc, &area);
}

void buildBat(lv_point_t* out, float scale, int cx, int cy) {
    for (int i = 0; i < HALF_COUNT; i++) {
        out[i].x = cx + std::lroundf((BAT_HALF[i].x - BAT_CX) * scale);
        out[i].y = cy + std::lroundf((BAT_HALF[i].y - BAT_CY) * scale);
    }
    for (int i = HALF_COUNT - 2, k = HALF_COUNT; i >= 1; i--, k++) { // mirror the other half
        out[k].x = cx - std::lroundf((BAT_HALF[i].x - BAT_CX) * scale);
        out[k].y = cy + std::lroundf((BAT_HALF[i].y - BAT_CY) * scale);
    }
}

void fillPolygon(lv_draw_ctx_t* draw_ctx, const lv_point_t* pts, int count, uint32_t color) {
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color = lv_color_hex(color);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.radius = 0;
    dsc.border_width = 0;

    int top = pts[0].y, bottom = pts[0].y;
    for (int i = 1; i < count; i++) {
        if (pts[i].y < top) top = pts[i].y;
        if (pts[i].y > bottom) bottom = pts[i].y;
    }

    for (int y = top; y <= bottom; y++) {
        int crossings[16];
        int found = 0;
        for (int i = 0; i < count; i++) {
            const lv_point_t& a = pts[i];
            const lv_point_t& b = pts[(i + 1) % count];
            const bool crosses = (a.y <= y && b.y > y) || (b.y <= y && a.y > y);
            if (!crosses || found >= 16) continue;
            const float howFar = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
            crossings[found++] = a.x + static_cast<int>(howFar * (b.x - a.x));
        }
        std::sort(crossings, crossings + found);
        for (int k = 0; k + 1 < found; k += 2)
            drawSpan(draw_ctx, &dsc, crossings[k], crossings[k + 1], y);
    }
}

void drawCornerBat(lv_event_t* e) {
    lv_draw_ctx_t* draw_ctx = lv_event_get_draw_ctx(e);
    lv_point_t bat[BAT_COUNT];
    buildBat(bat, CORNER_SCALE, CORNER_CX, CORNER_CY);
    fillPolygon(draw_ctx, bat, BAT_COUNT, GOLD);
}

// ---- selector behaviour ----

void onPrev(lv_event_t*) {
    if (!autonLocked) autonIndex = (autonIndex + AUTON_ROUTE_COUNT - 1) % AUTON_ROUTE_COUNT;
}

void onNext(lv_event_t*) {
    if (!autonLocked) autonIndex = (autonIndex + 1) % AUTON_ROUTE_COUNT;
}

// lock matters because the V5 screen is a touchscreen, a palm on the brain could change the route
void onLock(lv_event_t*) { autonLocked = !autonLocked; }

void uiCb(lv_timer_t*) {
    const AutonRoute& slot = AUTON_ROUTES[autonIndex];
    lv_label_set_text(autonLabel, slot.name);

    char buf[64];
    snprintf(buf, sizeof(buf), "%d / %d    %s%s", autonIndex + 1, AUTON_ROUTE_COUNT, nameOf(slot.color),
             autonLocked ? "    LOCKED" : "");
    lv_label_set_text(slotLabel, buf);

    lv_label_set_text(lockBtnLabel, autonLocked ? "UNLOCK" : "LOCK");
}

// ---- building the objects ----

lv_obj_t* makeLabel(lv_obj_t* parent, int x, int y, const lv_font_t* font, uint32_t color) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);
    lv_label_set_text(label, "");
    return label;
}

// dark button with a gold border and gold text
void makeButton(lv_obj_t* parent, int x, const char* text, lv_event_cb_t cb, lv_obj_t** labelOut) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 148, 44);
    lv_obj_set_pos(btn, x, 188);
    lv_obj_set_style_bg_color(btn, lv_color_hex(BTN_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(GOLD), LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(GOLD), LV_PART_MAIN);
    lv_obj_center(label);
    if (labelOut != nullptr) *labelOut = label;
}

} // namespace

// ---- what the rest of the robot calls ----

void screenShowSelector() {
    if (selectorObj != nullptr) return;

    // full-screen black panel, the gold corner bat draws on top of it
    selectorObj = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(selectorObj);
    lv_obj_set_size(selectorObj, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(selectorObj, 0, 0);
    lv_obj_set_style_bg_color(selectorObj, lv_color_hex(BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(selectorObj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(selectorObj, drawCornerBat, LV_EVENT_DRAW_MAIN, nullptr);

    lv_obj_t* team = makeLabel(selectorObj, 20, 18, &lv_font_montserrat_20, GOLD);
    lv_label_set_text(team, "393B");

    autonLabel = makeLabel(selectorObj, 20, 78, &lv_font_montserrat_24, WHITE); // the picked route
    slotLabel = makeLabel(selectorObj, 20, 120, &lv_font_montserrat_16, GRAY);  // slot number + color

    makeButton(selectorObj, 10, "< PREV", onPrev, nullptr);
    makeButton(selectorObj, 166, "LOCK", onLock, &lockBtnLabel);
    makeButton(selectorObj, 322, "NEXT >", onNext, nullptr);

    uiTimer = lv_timer_create(uiCb, 100, nullptr);
    uiCb(nullptr); // fill it in once now so it's never blank
}

const AutonRoute& selectedRoute() { return AUTON_ROUTES[autonIndex]; }

ToggleColor allianceColor() { return AUTON_ROUTES[autonIndex].color; }
