#pragma once

static constexpr float t = 0.25f;
static constexpr float l = 3.0f;

std::vector<glm::vec3> g_boxUnit = {
        {-1, -1, -1},
        {-1, -1,  1},
        {-1,  1,  1},
        {-1,  1, -1},

        {1,  1,  1},
        {1,  1, -1},
        {1, -1, -1},
        {1, -1,  1}
};


std::vector<glm::vec3> g_halfBoxUnit = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5,  0.5},
        {-0.5,  0.5,  0.5},
        {-0.5,  0.5, -0.5},

        {0.5,  0.5,  0.5},
        {0.5,  0.5, -0.5},
        {0.5, -0.5, -0.5},
        {0.5, -0.5,  0.5}
};

std::vector<glm::vec3> g_boxSmall = {
        {-t, -t, -t},
        {-t, -t,  t},
        {-t,  t,  t},
        {-t,  t, -t},

        { t, t,  t},
        { t, t, -t},
        { t,-t, -t},
        { t,-t,  t}
};

std::vector<glm::vec3> g_boxBeam = {
        {-l, -t, -t},
        {-l, -t,  t},
        {-l,  t,  t},
        {-l,  t, -t},

        { l, t,  t},
        { l, t, -t},
        { l,-t, -t},
        { l,-t,  t}
};

std::vector<glm::vec3> g_boxPlatform = {
        {-l, -t, -l},
        {-l, -t,  l},
        {-l,  t,  l},
        {-l,  t, -l},

        { l, t,  l},
        { l, t, -l},
        { l,-t, -l},
        { l,-t,  l}
};