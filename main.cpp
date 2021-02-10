#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

using namespace Eigen;


Matrix4f get_view_matrix(Vector3f eye_pos)
{
    Matrix4f view = Matrix4f::Identity();

    Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Matrix4f get_model_matrix(float angle)
{
    Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Matrix4f scale;
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
}

Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Matrix4f projection = Matrix4f::Identity();

    Matrix4f persp2ortho = Matrix4f::Identity();
    persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zFar * zNear,
        0, 0, 1, 0;

    float half_rad = eye_fov * 0.5 / 180 * MY_PI;

    float box_top = zNear * tan(half_rad);
    float box_bottom = -box_top;
    float box_right = box_top * aspect_ratio;
    float box_left = -box_right;

    Matrix4f ortho_scale = Matrix4f::Identity();
    ortho_scale << 2.0 / (box_right - box_left), 0, 0, 0,
        0, -2.0 / (box_top - box_bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    Matrix4f ortho_trans = Matrix4f::Identity();
    ortho_trans << 1, 0, 0, -(box_left + box_right) / 2,
        0, 1, 0, -(box_top + box_bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Matrix4f ortho = ortho_scale * ortho_trans;
    projection = ortho * persp2ortho;

    return projection;
}

Vector3f vertex_shader(const vertex_shader_payload &payload)
{
    return payload.position;
}

Vector3f normal_fragment_shader(const fragment_shader_payload &payload)
{
    Vector3f return_color = (payload.normal.head<3>().normalized() + Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Vector3f reflect(const Vector3f &vec, const Vector3f &axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Vector3f position;
    Vector3f intensity;
};

Vector3f texture_fragment_shader(const fragment_shader_payload &payload)
{
    Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Vector3f ka = Vector3f(0.005, 0.005, 0.005);
    Vector3f kd = texture_color / 255.f;
    Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {600, 200, 300}};
    auto l2 = light{{-20, 20, 0}, {200, 300, 600}};

    std::vector<light> lights = {l1, l2};
    Vector3f amb_light_intensity{10, 10, 10};
    Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Vector3f color = texture_color;
    Vector3f point = payload.view_pos;
    Vector3f normal = payload.normal;

    Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
        Vector3f light_dir = light.position - point;
        Vector3f view_dir = eye_pos - point;
        float r = light_dir.dot(light_dir);
        // ambient
        Vector3f La = ka.cwiseProduct(amb_light_intensity);
        // diffuse
        Vector3f Ld = kd.cwiseProduct(light.intensity / r);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir.normalized()));
        // specular
        Vector3f h = (light_dir.normalized() + view_dir.normalized()).normalized();
        Vector3f Ls = ks.cwiseProduct(light.intensity / r);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(h)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}

int main(int argc, const char **argv)
{
    std::vector<Triangle *> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";

    // Load .obj File
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    auto texture_path =  "spot_texture.png";

    for (auto mesh : Loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle *t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(512, 512);

    r.set_texture(Texture(obj_path + texture_path));

    std::function<Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
    r.set_texture(Texture(obj_path + texture_path));

    Vector3f eye_pos = {0, 0, 10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        angle += 1;
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(512, 512, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(1);
    }
    return 0;
}
