#include "route_image.hpp"
#include "route_config.hpp"

#pragma section all "cpu0_dsram"

#define ROUTE_EDGE_LEFT_FOUND   (0x01U)
#define ROUTE_EDGE_RIGHT_FOUND  (0x02U)

static uint8 ramp_confirm_count = 0;
static uint8 ramp_timeout_count = 0;
static uint8 ramp_exit_count = 0;

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

static int route_contrast(uint8 inside, uint8 outside)
{
    const int sum = (int)inside + (int)outside;
    if (sum == 0)
    {
        return 0;
    }
    return ((int)inside - (int)outside) * 200 / sum;
}

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

static void route_trace_edges(const uint8 *image,
    route_image_result_t *result)
{
    int predicted_center = result->reference_col;
    int estimated_width = MT9V03X_W - 20;

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

        int center = predicted_center;
        if (left_found && right_found
            && right - left >= ROUTE_MIN_TRACK_WIDTH)
        {
            estimated_width = right - left;
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
        result->center_line[row] = (uint8)center;
        result->track_width[row] = (uint8)route_limit_int(right - left,
            0, MT9V03X_W - 1);
        predicted_center = center;
    }
}

static void route_calculate_steering_error(route_image_result_t *result)
{
    int weighted_error = 0;
    int weight_sum = 0;
    int usable_rows = 0;
    const int image_center = MT9V03X_W / 2;
    int first_row = result->reference_row + 5;

    if (first_row < 30)
    {
        first_row = 30;
    }

    for (int row = first_row; row <= 100 && row < MT9V03X_H;
        row += 5)
    {
        if (result->edge_flags[row] == 0)
        {
            continue;
        }

        // Far rows receive more weight so the vehicle turns before the bend.
        const int weight = 1 + (100 - row) / 20;
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

static void route_update_ramp_state(route_image_result_t *result)
{
    const uint8 both_edges = ROUTE_EDGE_LEFT_FOUND
        | ROUTE_EDGE_RIGHT_FOUND;
    if (result->edge_flags[40] != both_edges
        || result->edge_flags[50] != both_edges)
    {
        result->ramp_active = (result->ramp_state >= 1
            && result->ramp_state <= 4) ? 1U : 0U;
        return;
    }

    const int delta = (int)result->track_width[50]
        - (int)result->track_width[40];
    result->ramp_width_delta = (int16)delta;

    switch (result->ramp_state)
    {
        case 0:
            if (delta > 3 && delta < 9 && result->track_valid)
            {
                if (++ramp_confirm_count > 5)
                {
                    result->ramp_state = 1;
                    ramp_confirm_count = 0;
                    ramp_timeout_count = 0;
                }
            }
            else
            {
                ramp_confirm_count = 0;
            }
            break;

        case 1:
            if (++ramp_timeout_count >= 100)
            {
                result->ramp_state = 0;
                ramp_timeout_count = 0;
            }
            else if (delta > 15)
            {
                result->ramp_state = 2;
            }
            break;

        case 2:
            if (delta > 5 && delta < 9)
            {
                result->ramp_state = 3;
            }
            break;

        case 3:
            if (delta > 10)
            {
                result->ramp_state = 4;
                ramp_confirm_count = 0;
            }
            break;

        case 4:
            if (delta > 10)
            {
                if (++ramp_confirm_count > 5)
                {
                    result->ramp_state = 5;
                    ramp_confirm_count = 0;
                    ramp_exit_count = 0;
                }
            }
            else
            {
                ramp_confirm_count = 0;
            }
            break;

        default:
            if (++ramp_exit_count >= 20)
            {
                result->ramp_state = 0;
                ramp_exit_count = 0;
            }
            break;
    }

    result->ramp_active = (result->ramp_state >= 1
        && result->ramp_state <= 4) ? 1U : 0U;
}

void route_image_reset(route_image_result_t *result)
{
    result->white_min = ROUTE_BLACK_FLOOR;
    result->white_max = 255;
    result->reference_col = MT9V03X_W / 2;
    result->reference_row = MT9V03X_H - 1;
    result->steering_error = 0;
    result->ramp_width_delta = 0;
    result->track_valid = 0;
    result->ramp_state = 0;
    result->ramp_active = 0;

    for (int row = 0; row < MT9V03X_H; ++row)
    {
        result->left_edge[row] = 0;
        result->right_edge[row] = MT9V03X_W - 1;
        result->center_line[row] = MT9V03X_W / 2;
        result->track_width[row] = 0;
        result->edge_flags[row] = 0;
    }

    ramp_confirm_count = 0;
    ramp_timeout_count = 0;
    ramp_exit_count = 0;
}

void route_image_process(const uint8 *image, route_image_result_t *result)
{
    const uint8 previous_ramp_state = result->ramp_state;

    for (int row = 0; row < MT9V03X_H; ++row)
    {
        result->left_edge[row] = 0;
        result->right_edge[row] = MT9V03X_W - 1;
        result->center_line[row] = MT9V03X_W / 2;
        result->track_width[row] = 0;
        result->edge_flags[row] = 0;
    }
    result->steering_error = 0;
    result->ramp_width_delta = 0;
    result->track_valid = 0;
    result->ramp_state = previous_ramp_state;

    route_get_white_reference(image, result);
    route_find_reference_column(image, result);
    route_trace_edges(image, result);
    route_calculate_steering_error(result);
    route_update_ramp_state(result);
}

#pragma section all restore
