// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    
    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
    0,1,0,-eye_pos[1],
    0,0,1,-eye_pos[2],
    0,0,0,1;
    
    view = translate*view;
    
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    float theta = (rotation_angle / 180.0) * MY_PI;//角度转弧度
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();//新建一个4x4的矩阵

    model << std::cos(theta), -std::sin(theta), 0, 0,
         std::sin(theta), std::cos(theta), 0, 0,
         0, 0, 1.0, 0,
         0, 0, 0, 1.0;
    //PS：绕X、Z轴旋转的旋转矩阵同理，绕Y轴的旋转矩阵略有不同，sin和-sin的位置会互换
    
    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis,float angle)
{
    float angle_x,angle_y,angle_z;
    float length = sqrt(axis.x() * axis.x() + axis.y()*axis.y()+axis.z()*axis.z());
    angle_x = std::acos(axis.x()/length);
    angle_y = std::acos(axis.y()/length);
    angle_z = std::acos(axis.z()/length);
    Eigen::Matrix4f m1,m2,m3  = Eigen::Matrix4f::Identity();
    m1<<1,0,0,0,0,cos(angle_x),-sin(angle_x),0,0,sin(angle_x),cos(angle_x),0,0,0,0,1;
    m2<<cos(angle_y),0,sin(angle_y),0,0,1,0,0,-sin(angle_y),0,cos(angle_y),0,0,0,0,1;
    m3<<cos(angle_z),-sin(angle_z),0,0,sin(angle_z),cos(angle_z),0,0,0,0,1,0,0,0,0,1;

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation =m3*m2*m1*Eigen::Matrix4f::Identity();
    return rotation;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    //Eigen::Matrix4f::Identity() 初始化为单位矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //透视图，近大远小，是个视锥  此矩阵是一个公式
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//将透视矩阵挤压成正交矩阵
    P2O << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, - zFar* zNear,
        0, 0, 1.0, 0;

    float halfEyeAngelRadian = (eye_fov / 2.0 / 180.0) * MY_PI; //视角的一半
    float y_top = -zNear * std::tan(halfEyeAngelRadian);//y轴正方向值 = 显示视口的一半高度 zNear是负值！
    float x_left = -y_top * aspect_ratio;//x轴负方向值 = 显示视口的一半宽度
    float y_down = -y_top;
    float x_right = -x_left;

    //构造缩放矩阵，使视口大小等同窗口大小
    Eigen::Matrix4f scaleMat = Eigen::Matrix4f::Identity();
    scaleMat << 2 / (x_right - x_left), 0, 0, 0,            //将中心视为原点，则窗口的三维方向值域均为[-1,1]
                0, 2 / (y_top - y_down), 0, 0,                //缩放的倍数为 期望值/当前值
                0, 0, 2 / (zNear - zFar), 0,                //所以缩放的倍数为 (1+1)/某一维度的当前值
                0, 0, 0, 1;

    //构造平移矩阵，将视口左下角移动到原点
    Eigen::Matrix4f translateMat = Eigen::Matrix4f::Identity();
    
    //左下角的点原本为 （x_left,y_down，zNear）
    //注意！此时已经经过了缩放，所以左下角的点的位置已经变化
    //左下角的点现在为 （-1，-1，zNear）
    //即其实可以不用管x和y轴，比较尺寸已经和窗口匹配了
    //但网上其他人却还是右下方注释的那样写的，左侧+右侧或者上侧+下侧，结果不都是0么？
    translateMat << 1, 0, 0, 0,                    //-(x_left+x_right)/2
            0, 1, 0, 0,                    //-(y_top+y_down)/2
            0, 0, 1, -(zNear+zFar)/2,
            0, 0, 0, 1;
    //注意！此处矩阵相乘，右结合率，必须先压缩矩阵，再缩放，最后平移，顺序一定不可以改！
    projection = translateMat * scaleMat * P2O;
    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    
    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }
    
    rst::rasterizer r(700, 700);
    
    Eigen::Vector3f eye_pos = {0,0,5};
    
//
//    std::vector<Eigen::Vector3f> pos
//    {
//        {2, 0, -2},
//        {0, 2, -2},
//        {-2, 0, -2},
//        {3.5, -1, -5},
//        {2.5, 1.5, -5},
//        {-1, 0.5, -5}
//    };
//
//    std::vector<Eigen::Vector3i> ind
//    {
//        {0, 1, 2},
//        {3, 4, 5}
//    };
//
//    std::vector<Eigen::Vector3f> cols
//    {
//        {217.0, 238.0, 185.0},
//        {217.0, 238.0, 185.0},
//        {217.0, 238.0, 185.0},
//        {185.0, 217.0, 238.0},
//        {185.0, 217.0, 238.0},
//        {185.0, 217.0, 238.0}
//    };
    
    std::vector<Eigen::Vector3f> pos {
        {-1, 1, 0},
        {-1, 0, 0},
        {1, 1, 1},

        {1, 1, 0},
        {0, 1, 0},
        {1, -1, 1},

        {1, -1, 0},
        {1, 0, 0},
        {-1, -1, 1},

        {-1, -1, 0},
        {0, -1, 0},
        {-1, 1, 1}
    };

    std::vector<Eigen::Vector3i> ind {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8},
        {9, 10, 11}
    };

    std::vector<Eigen::Vector3f> cols {
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},

        {217.0, 185.0, 238.0},
        {217.0, 185.0, 238.0},
        {217.0, 185.0, 238.0},

        {238.0, 185.0, 217.0},
        {238.0, 185.0, 217.0},
        {238.0, 185.0, 217.0},

        {238.0, 217.0, 185.0},
        {238.0, 217.0, 185.0},
        {238.0, 217.0, 185.0}
    };
    
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);
    
    int key = 0;
    int frame_count = 0;
    
    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        
        cv::imwrite(filename, image);
        
        return 0;
    }
    
    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);
        
        std::cout << "frame count: " << frame_count++ << '\n';
    }
    
    return 0;
}
// clang-format on
