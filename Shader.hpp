//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"

using namespace Eigen;


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Vector3f& col, const Vector3f& nor,const Vector2f& tc, Texture* tex) :
         color(col), normal(nor), tex_coords(tc), texture(tex) {}


    Vector3f view_pos;
    Vector3f color;
    Vector3f normal;
    Vector2f tex_coords;
    Texture* texture;
};

struct vertex_shader_payload
{
    Vector3f position;
};

#endif //RASTERIZER_SHADER_H
