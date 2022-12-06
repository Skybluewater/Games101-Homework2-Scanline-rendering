// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);
    
    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);
    
    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);
    
    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static int cross(const Eigen::Vector2f &p, const Eigen::Vector2f &q, const Eigen::Vector2f &r) {
    return (q.x() - p.x()) * (r.y() - q.y()) - (q.y() - p.y()) * (r.x() - q.x());
}

static bool insideTriangle(double x, double y, const Vector3f* _v) {
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f p;
    p << x, y;
    Eigen::Vector2f a = {_v[0].x(), _v[0].y()}, b = {_v[1].x(), _v[1].y()}, c = {_v[2].x(), _v[2].y()};
    return (cross(a, b, p) <= 0 && cross(b, c, p) <= 0 && cross(c, a, p) <= 0) || (cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];
    
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;
    
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }
        
        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];
        
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        rasterize_metric(t);
    }
}

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;
    
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

// return sequence in higher y, lower x
bool cmp(Eigen::Vector3f a, Eigen::Vector3f b) {
    if (a.y() == b.y()) {
        return a.x() < b.x();
    }
    return a.y() > b.y();
}

void rst::rasterizer::rasterize_metric(const Triangle& t) {
    auto v = t.toVector4();
    std::vector<Eigen::Vector3f> vc {
        {v[0].x(), v[0].y(), v[0].z()},
        {v[1].x(), v[1].y(), v[1].z()},
        {v[2].x(), v[2].y(), v[2].z()}
    };
    std::sort(vc.begin(), vc.end(), cmp);
    // v1 最高 v2 其次 v3 最矮
    Eigen::Vector3f v1 = vc[0], v2 = vc[1], v3 = vc[2];
    if (v2.y() == v3.y()) {
        // v1 > v2 == v3
        fillBottomFlatTriangle(v2, v3, v1, t.getColor());
    } else if (v1.y() == v2.y()) {
        // v1 == v2 > v3
        fillTopFlatTriangle(v1, v2, v3, t.getColor());
    } else {
        // get x
        float v4x = v1.x() + ((float)(v2.y() - v1.y()) / (float)(v3.y() - v1.y())) * (v3.x() - v1.x());
        // get z
        auto [alpha, beta, gamma] = computeBarycentric2D(v4x, v2.y(), t.v);
        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        z_interpolated *= w_reciprocal;
        // initialize v4
        Eigen::Vector3f v4 = {v4x, v2.y(), z_interpolated};
        // v1 > v2 == v4
        std::vector<Eigen::Vector3f> vc_up {
            v1,
            v2,
            v4
        };
        std::sort(vc_up.begin(), vc_up.end(), cmp);
        std::vector<Eigen::Vector3f> vc_down {
            v2,
            v4,
            v3
        };
        std::sort(vc_down.begin(), vc_down.end(), cmp);
        fillBottomFlatTriangle(vc_up[1], vc_up[2], vc_up[0], t.getColor());
        // v2 == v4 > v3
        fillTopFlatTriangle(vc_down[0], vc_down[1], vc_down[2], t.getColor());
    }
}

void rst::rasterizer::update_depth(int i, int j, int z, Eigen::Vector3f color) {
    depth_buf[get_index(i, j)] = z;
    Eigen::Vector3f pnt;
    pnt << static_cast<float>(i), static_cast<float>(j), z;
    set_pixel(pnt, color);
}

void rst::rasterizer::update_super_depth(int x, int y, int z, int i, Eigen::Vector3f color) {
    super_depth_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = z;
    super_frame_buf[get_super_index(x * 2 + i % 2, y * 2 + i / 2)] = color;
    Eigen::Vector3f pnt;
    Vector3f color_update = (super_frame_buf[get_super_index(x * 2 , y * 2)]+ super_frame_buf[get_super_index(x * 2 + 1, y * 2)]+
                             super_frame_buf[get_super_index(x * 2, y * 2 + 1)]+ super_frame_buf[get_super_index(x * 2 + 1, y * 2 + 1)])/ 4;
    pnt << static_cast<float>(x), static_cast<float>(y), z;
    set_pixel(pnt, color_update);
}

void rst::rasterizer::fillBottomFlatTriangle(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f color) {
    // invslope1 > 0
    // invslope2 < 0
    float invslope1 = (v3.x() - v1.x()) / (v3.y() - v1.y());
    float invslope2 = (v3.x() - v2.x()) / (v3.y() - v2.y());
    
    float curx1 = v3.x();
    float curx2 = v3.x();
    
    Eigen::Vector3f v[3];
    v[0] << v1.x(), v1.y(), v1.z();
    v[1] << v2.x(), v2.y(), v2.z();
    v[2] << v3.x(), v3.y(), v3.z();
    for (int scanlineY = v3.y(); scanlineY >= v1.y() - 1; scanlineY--)
    {
        draw_line(curx1, curx2, scanlineY, v, color);
        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

void rst::rasterizer::fillTopFlatTriangle(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f color) {
    // invslope1 < 0, invslope2 > 0
    float invslope1 = (v3.x() - v1.x()) / (v3.y() - v1.y());
    float invslope2 = (v3.x() - v2.x()) / (v3.y() - v2.y());
    
    float curx1 = v3.x();
    float curx2 = v3.x();
    
    Eigen::Vector3f v[3];
    v[0] << v3.x(), v3.y(), v3.z();
    v[1] << v2.x(), v2.y(), v2.z();
    v[2] << v1.x(), v1.y(), v1.z();
    for (int scanlineY = v3.y(); scanlineY <= v1.y() + 1; scanlineY++)
    {
        draw_line(curx1, curx2, scanlineY, v, color);
        curx1 += invslope1;
        curx2 += invslope2;
    }
}

void rst::rasterizer::draw_line(float x1, float x2, int y, Eigen::Vector3f* v, Eigen::Vector3f color) {
    bool MSAA = true;//MSAA是否启用
    int left_edge = static_cast<int>(x1), right_edge = static_cast<int>(x2 + 1);
    if (MSAA) {
        std::vector<Eigen::Vector2f> pos {
            //对一个像素分割四份 当然你还可以分成4x4 8x8等等甚至你还可以为了某种特殊情况设计成不规则的图形来分割单元
            {0.25,0.25},                //左下
            {0.75,0.25},                //右下
            {0.25,0.75},                //左上
            {0.75,0.75}                 //右上
        };
        for(float i = left_edge; i <= right_edge; i++){
            float mindep = INT_MAX, eid = get_index(i, y) * 4;
            for(int MSAA_4 = 0; MSAA_4 < 4; MSAA_4++){
                if(insideTriangle(i + pos[MSAA_4][0], y + pos[MSAA_4][1], v)){
                    // If so, use the following code to get the interpolated z value.
                    auto[alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i + pos[MSAA_4][0]), static_cast<float>(y + pos[MSAA_4][1]), v);
                    float w_reciprocal = 1.0/(alpha / 1.f + beta / 1.f + gamma / 1.f);
                    float z_interpolated = alpha * v[0].z() / 1.f + beta * v[1].z() / 1.f + gamma * v[2].z() / 1.f;
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < super_depth_buf[eid + MSAA_4]){
                        super_depth_buf[eid + MSAA_4] = z_interpolated;
                        super_frame_buf[eid + MSAA_4] = color / 4;
                    }
                    mindep = std::min(super_depth_buf[eid + MSAA_4], mindep);
                }
            }
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            Vector3f color_update = super_frame_buf[eid] + super_frame_buf[eid + 1] + super_frame_buf[eid + 2] + super_frame_buf[eid + 3];
            set_pixel({i, y, 1}, color_update);
            depth_buf[get_index(i, y)] = std::min(depth_buf[get_index(i, y)], mindep);
        }
    }
    else {
        for (int i = left_edge; i <= right_edge; ++i) {
            if(insideTriangle(i, y, v)){
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i + 0.5), static_cast<float>(y + 0.5), v);
                float w_reciprocal = 1.0/(alpha / 1.f + beta / 1.f + gamma / 1.f);
                float z_interpolated = alpha * v[0].z() / 1.f + beta * v[1].z() / 1.f + gamma * v[2].z() / 1.f;
                z_interpolated *= w_reciprocal;
                if(z_interpolated < depth_buf[get_index(i, y)]){
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    set_pixel({i, y, 1}, color);
                    depth_buf[get_index(i, y)] = z_interpolated;
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

int rst::rasterizer::get_super_index(int x, int y) {
    return (height * 2 - 1 - y) * width * 2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
    
}

// clang-format on
