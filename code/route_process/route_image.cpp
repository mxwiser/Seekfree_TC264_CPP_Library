#include "route_image.hpp"
#include "route_config.hpp"

#pragma section all "cpu0_dsram"

#define ROUTE_EDGE_LEFT_FOUND   (0x01U)
#define ROUTE_EDGE_RIGHT_FOUND  (0x02U)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将整数限制在指定的闭区间内
// 参数说明     value           待限制的数值
// 参数说明     minimum         允许的最小值
// 参数说明     maximum         允许的最大值
// 返回参数     int             限幅后的数值
//-------------------------------------------------------------------------------------------------------------------
static int route_limit_int(int value, int minimum, int maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算赛道内外两个像素的归一化亮度对比度
// 参数说明     inside          赛道内部像素灰度值
// 参数说明     outside         赛道外部像素灰度值
// 返回参数     int             归一化对比度，正值表示内部比外部更亮
// 备注信息     使用差值除以灰度和，可减小整体环境亮度变化造成的影响
//-------------------------------------------------------------------------------------------------------------------
static int route_contrast(uint8 inside, uint8 outside)
{
    const int sum = (int)inside + (int)outside;
    if (sum == 0)
    {
        return 0;
    }
    return ((int)inside - (int)outside) * 200 / sum;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据图像底部若干行的平均灰度生成白色赛道判定阈值
// 参数说明     image           MT9V03X 灰度图像首地址
// 参数说明     result          图像处理结果，写入 white_min 和 white_max
// 返回参数     void
// 备注信息     阈值会被限制在 ROUTE_BLACK_FLOOR 到 255 之间
//-------------------------------------------------------------------------------------------------------------------
static void route_get_white_reference(const uint8 *image,
    route_image_result_t *result)
{
    uint32 sum = 0;
    const int first_row = MT9V03X_H - ROUTE_REFERENCE_ROWS;

    for (int row = first_row; row < MT9V03X_H; ++row)
    {
        const uint8 *line = image + row * MT9V03X_W;
        for (int col = 0; col < MT9V03X_W; ++col)
        {
            sum += line[col];
        }
    }

    const int point_count = ROUTE_REFERENCE_ROWS * MT9V03X_W;
    const int reference = (int)(sum / (uint32)point_count);
    result->white_min = (uint8)route_limit_int(
        reference * ROUTE_WHITE_MIN_RATIO_X10 / 10,
        ROUTE_BLACK_FLOOR, 255);
    result->white_max = (uint8)route_limit_int(
        reference * ROUTE_WHITE_MAX_RATIO_X10 / 10,
        ROUTE_BLACK_FLOOR, 255);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像中寻找最适合作为边线追踪起点的参考列和参考行
// 参数说明     image           MT9V03X 灰度图像首地址
// 参数说明     result          图像处理结果，读取白色阈值并写入参考坐标
// 返回参数     void
// 备注信息     从图像底部向上寻找灰度突变；候选位置相同时优先选择靠近图像中心的列
//-------------------------------------------------------------------------------------------------------------------
static void route_find_reference_column(const uint8 *image,
    route_image_result_t *result)
{
    const int center = MT9V03X_W / 2;
    int best_col = center;
    int best_row = MT9V03X_H - 1;
    int best_center_distance = MT9V03X_W;

    for (int col = ROUTE_CONTRAST_OFFSET;
        col < MT9V03X_W - ROUTE_CONTRAST_OFFSET;
        col += ROUTE_CONTRAST_OFFSET)
    {
        int remote_row = ROUTE_CONTRAST_OFFSET;
        for (int row = MT9V03X_H - 1;
            row > ROUTE_CONTRAST_OFFSET;
            row -= ROUTE_CONTRAST_OFFSET)
        {
            const uint8 current = image[row * MT9V03X_W + col];
            const uint8 ahead = image[(row - ROUTE_CONTRAST_OFFSET)
                * MT9V03X_W + col];

            if (current < result->white_min)
            {
                remote_row = row;
                break;
            }

            if (ahead <= result->white_max
                && route_contrast(current, ahead)
                    > ROUTE_CONTRAST_THRESHOLD)
            {
                remote_row = row;
                break;
            }
        }

        const int center_distance = (col > center)
            ? (col - center) : (center - col);
        if (remote_row < best_row
            || (remote_row == best_row
                && center_distance < best_center_distance))
        {
            best_row = remote_row;
            best_col = col;
            best_center_distance = center_distance;
        }
    }

    result->reference_col = (uint8)route_limit_int(best_col,
        ROUTE_CONTRAST_OFFSET,
        MT9V03X_W - ROUTE_CONTRAST_OFFSET - 1);
    result->reference_row = (uint8)route_limit_int(best_row,
        ROUTE_CONTRAST_OFFSET, MT9V03X_H - 20);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在当前图像行内寻找一个属于白色赛道区域的搜索种子点
// 参数说明     line            当前图像行的首地址
// 参数说明     predicted       根据上一行中心位置预测的种子列
// 参数说明     white_min       白色赛道的最低灰度阈值
// 返回参数     int             找到时返回种子列，未找到时返回 -1
// 备注信息     先检查预测位置，再在 ROUTE_SEED_SEARCH_RANGE 范围内向左右交替搜索
//-------------------------------------------------------------------------------------------------------------------
static int route_find_white_seed(const uint8 *line, int predicted,
    uint8 white_min)
{
    predicted = route_limit_int(predicted, ROUTE_CONTRAST_OFFSET,
        MT9V03X_W - ROUTE_CONTRAST_OFFSET - 1);
    if (line[predicted] >= white_min)
    {
        return predicted;
    }

    for (int offset = 1; offset <= ROUTE_SEED_SEARCH_RANGE; ++offset)
    {
        const int left = predicted - offset;
        const int right = predicted + offset;
        if (left >= ROUTE_CONTRAST_OFFSET && line[left] >= white_min)
        {
            return left;
        }
        if (right < MT9V03X_W - ROUTE_CONTRAST_OFFSET
            && line[right] >= white_min)
        {
            return right;
        }
    }
    return -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从白色种子点向左搜索赛道左边界
// 参数说明     line            当前图像行的首地址
// 参数说明     seed            当前行的白色种子列
// 参数说明     result          图像处理结果，提供动态白色阈值
// 参数说明     found           输出边界有效标志，1 表示找到，0 表示未找到
// 返回参数     int             左边界所在列；未找到时返回图像最左列
//-------------------------------------------------------------------------------------------------------------------
static int route_find_left_edge(const uint8 *line, int seed,
    const route_image_result_t *result, uint8 *found)
{
    for (int col = seed; col >= ROUTE_CONTRAST_OFFSET; --col)
    {
        const uint8 inside = line[col];
        const uint8 outside = line[col - ROUTE_CONTRAST_OFFSET];
        if (inside < result->white_min)
        {
            *found = 1;
            return col + 1;
        }
        if (outside < result->white_min
            || (outside <= result->white_max
                && route_contrast(inside, outside)
                    > ROUTE_CONTRAST_THRESHOLD))
        {
            *found = 1;
            return col;
        }
    }
    *found = 0;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从白色种子点向右搜索赛道右边界
// 参数说明     line            当前图像行的首地址
// 参数说明     seed            当前行的白色种子列
// 参数说明     result          图像处理结果，提供动态白色阈值
// 参数说明     found           输出边界有效标志，1 表示找到，0 表示未找到
// 返回参数     int             右边界所在列；未找到时返回图像最右列
//-------------------------------------------------------------------------------------------------------------------
static int route_find_right_edge(const uint8 *line, int seed,
    const route_image_result_t *result, uint8 *found)
{
    const int last = MT9V03X_W - ROUTE_CONTRAST_OFFSET - 1;
    for (int col = seed; col <= last; ++col)
    {
        const uint8 inside = line[col];
        const uint8 outside = line[col + ROUTE_CONTRAST_OFFSET];
        if (inside < result->white_min)
        {
            *found = 1;
            return col - 1;
        }
        if (outside < result->white_min
            || (outside <= result->white_max
                && route_contrast(inside, outside)
                    > ROUTE_CONTRAST_THRESHOLD))
        {
            *found = 1;
            return col;
        }
    }
    *found = 0;
    return MT9V03X_W - 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从图像底部向参考行逐行追踪左右边线并计算赛道中心线
// 参数说明     image           MT9V03X 灰度图像首地址
// 参数说明     result          写入左右边线、中心线和边线有效标志
// 返回参数     void
// 备注信息     单侧边线使用赛道宽度补线；中心突变过大时丢弃该行，抑制噪点干扰
//-------------------------------------------------------------------------------------------------------------------
static void route_trace_edges(const uint8 *image,
    route_image_result_t *result)
{
    int predicted_center = result->reference_col;
    int estimated_width = MT9V03X_W - 20;
    uint8 center_initialized = 0;

    for (int row = MT9V03X_H - 1;
        row >= (int)result->reference_row; --row)
    {
        const uint8 *line = image + row * MT9V03X_W;
        const int seed = route_find_white_seed(line, predicted_center,
            result->white_min);
        if (seed < 0)
        {
            continue;
        }

        uint8 left_found = 0;
        uint8 right_found = 0;
        const int left = route_find_left_edge(line, seed, result,
            &left_found);
        const int right = route_find_right_edge(line, seed, result,
            &right_found);

        int center = predicted_center;
        if (left_found && right_found
            && right - left >= ROUTE_MIN_TRACK_WIDTH)
        {
            center = (left + right) / 2;
        }
        else if (left_found && !right_found)
        {
            center = left + estimated_width / 2;
        }
        else if (!left_found && right_found)
        {
            center = right - estimated_width / 2;
        }
        else
        {
            continue;
        }

        center = route_limit_int(center, 0, MT9V03X_W - 1);
        const int center_jump = center > predicted_center
            ? center - predicted_center : predicted_center - center;
        if (center_initialized && center_jump > ROUTE_MAX_CENTER_JUMP)
        {
            continue;
        }

        if (left_found)
        {
            result->edge_flags[row] |= ROUTE_EDGE_LEFT_FOUND;
        }
        if (right_found)
        {
            result->edge_flags[row] |= ROUTE_EDGE_RIGHT_FOUND;
        }
        result->left_edge[row] = (uint8)left;
        result->right_edge[row] = (uint8)right;
        result->center_line[row] = (uint8)center;
        if (left_found && right_found)
        {
            estimated_width = right - left;
        }
        predicted_center = center;
        center_initialized = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据多行赛道中心计算用于舵机控制的横向偏差
// 参数说明     result          读取中心线和边线标志，写入 steering_error 与 track_valid
// 返回参数     void
// 备注信息     对较远处的图像行赋予更大权重，使车辆能够在进入弯道前提前转向
//-------------------------------------------------------------------------------------------------------------------
static void route_calculate_steering_error(route_image_result_t *result)
{
    int weighted_error = 0;
    int weight_sum = 0;
    int usable_rows = 0;
    const int image_center = MT9V03X_W / 2;
    int first_row = result->reference_row + 5;

    if (first_row < 25)
    {
        first_row = 25;
    }

    for (int row = first_row; row <= 90 && row < MT9V03X_H;
        row += 5)
    {
        if (result->edge_flags[row] == 0)
        {
            continue;
        }

        // Far rows receive more weight so the vehicle turns before the bend.
        const int weight = 1 + (90 - row) / 10;
        weighted_error += ((int)result->center_line[row] - image_center)
            * weight;
        weight_sum += weight;
        ++usable_rows;
    }

    if (usable_rows >= 4 && weight_sum > 0)
    {
        result->steering_error = (int16)route_limit_int(
            weighted_error / weight_sum, -60, 60);
        result->track_valid = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化图像处理结果
// 参数说明     result          需要初始化的图像处理结果结构体
// 返回参数     void
// 备注信息     应在首次处理摄像头图像前调用
//-------------------------------------------------------------------------------------------------------------------
void route_image_reset(route_image_result_t *result)
{
    result->white_min = ROUTE_BLACK_FLOOR;
    result->white_max = 255;
    result->reference_col = MT9V03X_W / 2;
    result->reference_row = MT9V03X_H - 1;
    result->steering_error = 0;
    result->track_valid = 0;

    for (int row = 0; row < MT9V03X_H; ++row)
    {
        result->left_edge[row] = 0;
        result->right_edge[row] = MT9V03X_W - 1;
        result->center_line[row] = MT9V03X_W / 2;
        result->edge_flags[row] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完成一帧赛道图像的全部处理流程
// 参数说明     image           MT9V03X 灰度图像首地址
// 参数说明     result          保存阈值、边线、中心线和转向偏差
// 返回参数     void
// 备注信息     依次执行动态阈值、参考点搜索、边线追踪和转向偏差计算
//-------------------------------------------------------------------------------------------------------------------
void route_image_process(const uint8 *image, route_image_result_t *result)
{
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        result->left_edge[row] = 0;
        result->right_edge[row] = MT9V03X_W - 1;
        result->center_line[row] = MT9V03X_W / 2;
        result->edge_flags[row] = 0;
    }
    result->steering_error = 0;
    result->track_valid = 0;

    route_get_white_reference(image, result);
    route_find_reference_column(image, result);
    route_trace_edges(image, result);
    route_calculate_steering_error(result);
}

#pragma section all restore
