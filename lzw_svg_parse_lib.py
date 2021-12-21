import numpy as np
import open3d as o3d
import svgelements as se
import math
import copy
def hello_lib():
    print("hello lzw's python lib")


class line():
    def __init__(self,start,end):
        self.start_=start
        self.end_=end
def process_poly_line(poly_line_ele):
    #process single poly line to return l_lines, list of many lines
    l_lines=[]
    print("this poly line have: ", len(poly_line_ele.points), "elements")
    if len(poly_line_ele.points)==2:
        start_point=poly_line_ele.points[0]
        end_point=poly_line_ele.points[1]
        one_line=line(start_point,end_point)
        l_lines.append(one_line)

    elif len(poly_line_ele.points)>2:
        for it_poly in range(len(poly_line_ele.points)-1):
            start_point = poly_line_ele.points[it_poly]
            end_point = poly_line_ele.points[it_poly]
            one_line = line(start_point, end_point)
            l_lines.append(one_line)
    print("return l_lines: ",len(l_lines)," lines ")
    return l_lines
#indices indicate which points need to be link
def vis_lines(np_pcd,indices,ori_frame_size):

    l_pcd_color = []
    for i in range(np_pcd.shape[0]):
        l_pcd_color.append([0, 0, 255])
    points_line_set = copy.deepcopy(np_pcd)
    #lines = []
    # a line link to point
    # for it_row in range(indices.shape[0]):
    #
    #     # need to plus [indices.shape[0]]
    #     lines_ele = [it_row, id + indices.shape[0]]
    #     lines.append(lines_ele)


    l_line_colors = [[1, 0, 0] for i in range(len(indices))]
    # points = o3d.utility.Vector3dVector(points)

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_line_set),
        lines=o3d.utility.Vector2iVector(indices),
    )
    line_set.colors = o3d.utility.Vector3dVector(l_line_colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    pcd.colors = o3d.utility.Vector3dVector(l_pcd_color)

    #axis_pcd is a frame

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=ori_frame_size, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([pcd] + [line_set]+[axis_pcd])

def main_draw_path(file):
    # np_test_pcd=np.array([[0.000,0.00,0.0],
    #                       [0,0.001,0],
    #                       [0.002,0.0005,0]])
    # lines_index=[[0,1] ,[0,2] ,[1,2]]
    # vis_lines(np_test_pcd,lines_index,ori_frame_size=0.0003)
    # it seems that Repetitive points and Repetitive lines don't have influence on
    # it seems that  too far away  from view point will  be truncated
    svg = se.SVG.parse(file)

    #global
    np_global_points=np.array([[0,0,0]])
    l_global_line_index=[]
    print(np_global_points.shape[0])
    print(np_global_points.shape[1])

    #np_global_points=np.row_stack((np_global_points, [1,1,1]))
    #print(np_global_points)
    #iter over groups
    path_count=0
    for it_svg in range(len(svg)):
        #in each groups, there are many basic elements
        #such as : polyline,path,circle,
        #path is the most complicated
        for it_group_ele in range(len(svg[it_svg])):
            #each single element
            print(type(svg[it_svg][it_group_ele]))
            #in each elements, there many subelements
            # if isinstance(svg[it_svg][it_group_ele],se.svgelements.Polyline)\
            #         and len(svg[it_svg][it_group_ele].points)>2:

            if isinstance(svg[it_svg][it_group_ele], se.svgelements.Polyline):
                print("-------start adding one polyline------")
                poly_line=svg[it_svg][it_group_ele]
                #process_poly_line(poly_line)
                print(" poly line  has: ",len(poly_line.points),"points ")
                print(" poly line  has: ", len(poly_line.points)-1, "lines ")
                print("points: ",'\n',poly_line.points)
                ori_row=np_global_points.shape[0]
                len_poly_line=len(poly_line.points)
                print("ori row num of gloabl points: ",ori_row)
                for it_pl_se in range(0,len(poly_line.points)):
                    np_global_points=np.row_stack((np_global_points, [poly_line.points[it_pl_se].x\
                        ,poly_line.points[it_pl_se].y,0]))
                    print("current row: ",np_global_points.shape[0])
                    if it_pl_se != len(poly_line.points)-1:
                        l_global_line_index.append([ori_row+it_pl_se,ori_row+it_pl_se+1])
                        print("added index: ",ori_row+it_pl_se,ori_row+it_pl_se+1)
                print("global point: ","\n",np_global_points,"\n","l_global_line_index:","\n",l_global_line_index)
                #vis_lines(np_global_points,l_global_line_index,600)
                print("-------end adding one polyline------")

            elif isinstance(svg[it_svg][it_group_ele], se.svgelements.Circle):
                print("-------start adding one polyline------")
                circle = svg[it_svg][it_group_ele]
                cx, cy, r = circle.cx, circle.cy, circle.implicit_r
                sample_num = 100
                sample_angle = 2 * math.pi / sample_num
                # np_points_set = np.array([0, 0, 0])
                # l_lines_set = []
                # it in 0,1,2,...,sample_num-1
                # so will sample sample_num times
                added_sample_num=0
                ori_row = np_global_points.shape[0]
                print("before adding circle points ,it has: ",ori_row,"rows")
                for it in range(0, sample_num):
                    np_global_points = np.row_stack((np_global_points, [cx + r * math.cos(it * sample_angle) \
                         , cy + r * math.sin(it * sample_angle), 0]))
                    added_sample_num+=1
                print("already added : ",added_sample_num, "circle point")
                # n sample will results n lines
                print("after adding circle points ,it has: ", np_global_points.shape[0], "rows")
                #to do: chang
                for it in range(0, sample_num - 1):
                    l_global_line_index.append([ori_row+it,ori_row+it+1])
                    print("added index: ",ori_row+it," ",ori_row+it+1)
                #
                #last line : last point connect to first point

                l_global_line_index.append([ori_row, ori_row+sample_num-1])
                print("added index: ", ori_row, ori_row+sample_num-1)
                #vis_lines(np_global_points, l_global_line_index, 600)

                print("-------end adding one polyline------")

            elif isinstance(svg[it_svg][it_group_ele], se.svgelements.Path):
                path_ele=svg[it_svg][it_group_ele]
                path_count+=1
    print("path count: ",path_count)
    vis_lines(np_global_points, l_global_line_index, 500)

def print_hi(file):
    svg = se.SVG.parse(file)
    l_svg_ele=list(svg.elements())

    o_svg=l_svg_ele[0]
    print(list(svg.elements()))
    print("type svg: ",type(svg))
    pll_group_ele=svg[2]
    path_ele=svg[1][1]
    path_ele_segments=path_ele._segments
    path_move_ele=path_ele_segments[0]
    path_line_ele = path_ele_segments[1]
    path_close_ele = path_ele_segments[3]
    print(path_ele)
    print("path ele : ",type(path_ele))


    polyline_ele=pll_group_ele[1]
    ployline_ele_points=polyline_ele.points
    if isinstance(path_ele,se.svgelements.Path):
        print("yes it is  a svgelements.svgelements.Path object")

    print(type(o_svg[1]))
    svg_ele=svg.elements()
    svg_object=svg.objects
    print("end progrom")

def demo_circle_poly_lines():
    cx,cy,r=1,1.5,1
    sample_num=100
    sample_angle=2*math.pi/sample_num
    np_points_set=np.array([0,0,0])
    l_lines_set=[]
    #it in 0,1,2,...,sample_num-1
    #so will sample sample_num times
    for it in range(0,sample_num):
        np_points_set = np.row_stack((np_points_set, [cx+r*math.cos(it*sample_angle) \
            ,cy+r*math.sin(it*sample_angle), 0]))
    #n sample will results n lines
    for it in range(0,sample_num-1):
        l_lines_set.append([it+1,it+2])

    #last line : last point connect to first point
    l_lines_set.append([1,np_points_set.shape[0]-1])
    print(np_points_set)
    print(l_lines_set)
    vis_lines(np_points_set,l_lines_set,1)

def circle_poly_line():
    pass





