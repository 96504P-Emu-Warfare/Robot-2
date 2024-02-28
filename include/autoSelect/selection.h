#pragma once

#include <string>

//selector configuration
#define HUE 192
#define DEFAULT 3
#define AUTONS "SAFE|4", "RISKY|6(R)", "RUSH|6"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
