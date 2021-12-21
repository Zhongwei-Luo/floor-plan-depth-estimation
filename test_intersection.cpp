//
// Created by lzw on 2021/11/19.
//


//already  tested
#include<cstdio>
#include<iostream>
#include<cmath>
#include<cstring>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


/*const double p=1e-10;
double x0,p0;
double k1,k2,b1,b2;
double D;*/


class abc_2d_line
{   public:
    pcl::PointXYZ start_;
    pcl::PointXYZ end_;
    double a0_;
    double b0_;
    double c0_;

    abc_2d_line(pcl::PointXYZ start,pcl::PointXYZ end,double a,double b,double c)
    {
        start_=start;
        end_=end;
         a0_=a;
         b0_=b;
         c0_=c;
    }

    abc_2d_line(pcl::PointXYZ start,pcl::PointXYZ end){
        start_=start;
        end_=end;

        a0_=start_.y-end_.y;
        b0_=end_.x-start_.x;
        c0_=start_.x*end_.y-start_.y*end_.x;
    }
    abc_2d_line() {};
};




int is_intersection(pcl::PointXYZ a,pcl::PointXYZ b,pcl::PointXYZ c,pcl::PointXYZ d);
pcl::PointXYZ calculate_intersection(pcl::PointXYZ &a,pcl::PointXYZ &b,pcl::PointXYZ &c,pcl::PointXYZ &d);
void duandian();
void draw_multi_trajectory(std::vector<abc_2d_line> &v_p,
                           std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses,
                           double axis_size);

int  main()
{

    duandian();

    return 0;
}

//given lines,and pose ,axis_size then drawing
void draw_multi_trajectory(std::vector<abc_2d_line> &v_p,
                           std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses,
                           double axis_size)
{

    pcl::visualization::PCLVisualizer vis_er;
    vis_er.setBackgroundColor(200, 200, 200);
    int line_num=0;
    for(uint it_s=0;it_s<v_p.size();it_s++)
    {  //double line_color=(it_tra+1)*color_div;
        double axis_len = 0.15;
        vis_er.addLine(v_p[it_s].start_, v_p[it_s].end_, 255, 0, 0, std::to_string (it_s)   );

    }



    //
    pcl::PointXYZ last_O_pos(0, 0, 0);
    double axis_len = axis_size;
    if(poses.size()!=0)
    {   for (int i = 0; i < poses.size(); i++)
        {
            pcl::PointXYZ O_pose;
            //每帧位姿的原点坐标只由变换矩阵中的平移向量得到
            O_pose.x = poses[i](0, 3);
            O_pose.y = poses[i](1, 3);
            O_pose.z = poses[i](2, 3);
            if (i > 0)
                vis_er.addLine(last_O_pos, O_pose, 0, 0, 0, "trac_" + std::to_string(i));

            pcl::PointXYZ X;

            Eigen::MatrixXd eg_x(4, 1);
            eg_x << axis_len, 0, 0, 1;
            Eigen::MatrixXd Xw(4, 1);
            Xw = poses[i] * (eg_x);
            X.x = Xw(0, 0);
            X.y = Xw(1, 0);
            X.z = Xw(2, 0);
            //drawing x axis,x axis->red
            vis_er.addLine(O_pose, X, 255, 0, 0, "X_" + std::to_string(i));

            pcl::PointXYZ Y;
            Eigen::MatrixXd eg_y(4, 1);
            eg_y << 0, axis_len, 0, 1;
            Eigen::MatrixXd Yw(4, 1);
            Yw = poses[i] * (eg_y);
            Y.x = Yw(0, 0);
            Y.y = Yw(1, 0);
            Y.z = Yw(2, 0);
            //drawing y axis,y axis is green
            vis_er.addLine(O_pose, Y, 0, 255, 0, "Y_" + std::to_string(i));

            pcl::PointXYZ Z;
            Eigen::MatrixXd eg_z(4, 1);
            eg_z << 0, 0, axis_len, 1;
            Eigen::MatrixXd Zw(4, 1);
            Zw = poses[i] * (eg_z);
            Z.x = Zw(0, 0);
            Z.y = Zw(1, 0);
            Z.z = Zw(2, 0);
            //drwaing z axis, z axis ->blue

            vis_er.addLine(O_pose, Z, 0, 0, 255, "Z_" + std::to_string(i));

            last_O_pos = O_pose;
        }}

    vis_er.spin();
};



void duandian()
{   pcl::PointXYZ a_s,a_e,b_s,b_e;
    pcl::PointXY a,b,c,d;


    /*a.x=0;a.y=0;b.x=2,b.y=2;
    c.x=0.5,c.y=1.5;d.x=0,d.y=2;*/

   /*a.x=0;a.y=0;b.x=2,b.y=2.5;
  c.x=0,c.y=1;d.x=2,d.y=3;*/

    a.x=0;a.y=0;b.x=2,b.y=2.5;
  c.x=0,c.y=1;d.x=2,d.y=3;


    a_s.x=a.x;a_s.y=a.y;
    a_e.x=b.x;a_e.y=b.y;


    b_s.x=c.x;b_s.y=c.y;

    b_e.x=d.x;b_e.y=d.y;

    abc_2d_line line_1,line_2;
    line_1.start_=a_s;
    line_1.end_=a_e;
    line_2.start_=b_s;
    line_2.end_=b_e;

    std::vector<abc_2d_line> v_lines;
    v_lines.push_back(line_1);
    v_lines.push_back(line_2);
    if(is_intersection(a_s,a_e,b_s,b_e))
    {
        cout<<"yes"<<endl;
        calculate_intersection(a_s,a_e,b_s,b_e);
    }

    else
        cout<<"no"<<endl;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    Eigen::Matrix4d ori_point;
    ori_point<<1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;
    poses.push_back(ori_point);

    draw_multi_trajectory(v_lines,poses,2);

}


pcl::PointXYZ calculate_intersection(pcl::PointXYZ &a,pcl::PointXYZ &b,pcl::PointXYZ &c,pcl::PointXYZ &d)
{
    abc_2d_line m(a,b),n(c,d);
   /* m.a0_=a.y-b.y;
    m.b0_=b.x-a.x;
    m.c0_=a.x*b.y-a.y*b.x;

    n.a0_=c.y-d.y;
    n.b0_=d.x-c.x;
    n.c0_=c.x*d.y-c.y*d.x;*/
    const double p=1e-10;
    double x0,p0;
    double k1,k2,b1,b2;
    double D;

    D=m.a0_*n.b0_-n.a0_*m.b0_;
    x0=(m.b0_*n.c0_-n.b0_*m.c0_)/D;
    p0=(n.a0_*m.c0_-m.a0_*n.c0_)/D;
    printf("交点坐标\n");
    printf("x: ");
    cout<<x0<<endl;
    printf("y: ");
    cout<<p0<<endl;

    pcl::PointXYZ intersection(x0,p0,0);
    return intersection;


}

int is_intersection(pcl::PointXYZ a,pcl::PointXYZ b,pcl::PointXYZ c,pcl::PointXYZ d)
{
    double p1,p2,p3,p4;
    const double p=1e-10;
    p1=(d.x-c.x)*(d.y-a.y)-(d.y-c.y)*(d.x-a.x);//DCxDA
    p2=(d.x-c.x)*(d.y-b.y)-(d.y-c.y)*(d.x-b.x);//DCxDB
    p3=(b.x-a.x)*(b.y-d.y)-(b.y-a.y)*(b.x-d.x);//BAxBD
    p4=(b.x-a.x)*(b.y-c.y)-(b.y-a.y)*(b.x-c.x);//BAxBC
    if(p1*p2<=p&&p3*p4<=p)//double判断有精度问题 注意
        return 1;
    return 0;
}