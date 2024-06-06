import cupy as cp

cube2equi_kernel = cp.RawKernel(r'''
extern "C" __global__
void cube2equi(const uchar3* front, const uchar3* back, const uchar3* top,
               const uchar3* bottom, const uchar3* left, const uchar3* right,
               uchar3* dst, int rows, int cols, int dims)
{
    float PI = 3.14159265358979323846;

    int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
    int dst_y = blockDim.y * blockIdx.y + threadIdx.y;

    if (dst_x < cols && dst_y < rows)
    {
        float v = 1.0 - ((float(dst_y)) / (dims * 2));
        float phi = v * PI;

        float u = (float(dst_x)) / (dims * 4);
        float theta = u * 2 * PI;

        float x = cos(theta) * sin(phi);
        float y = sin(theta) * sin(phi);
        float z = cos(phi);

        float a = fmaxf(fmaxf(fabsf(x), fabsf(y)), fabsf(z));

        float xx = x / a;
        float yy = y / a;
        float zz = z / a;

        float xPixel, yPixel, yTemp, imageSelect;
                                
        //front
        if (xx == 1)
        {
            xPixel =(tan(atan(y / x)) + 1) / 2 * dims;
            yTemp = (tan(atan(z / x)) + 1) / 2 * dims;
            imageSelect = 2;
        }
        //back
        else if (xx == -1) {
            xPixel = (tan(atan(y / x)) + 1) / 2 * dims;
            yTemp = (-tan(atan(z / x)) + 1) / 2 * dims;
            imageSelect = 1;
        }
        //top
        else if (zz == -1)
        {
            xPixel =(tan(atan(y / z)) + 1) / 2 * dims;
            yTemp = (tan(atan(x / z)) + 1) / 2 * dims;
            imageSelect = 3;
        }
        //bottom
        else if (zz == 1)
        {
            xPixel = (-tan(atan(y / z)) + 1) / 2 * dims;
            yTemp = (tan(atan(x / z)) + 1) / 2 * dims;
            imageSelect = 4;
        }                                
        //left
        else if (yy == -1)
        {
            xPixel = (-tan(atan(x / y)) + 1) / 2 * dims;
            yTemp = (-tan(atan(z / y)) + 1) / 2 * dims;
            imageSelect = 6;
        }

        //right
        else if (yy == 1)
        {
            xPixel = (-tan(atan(x / y)) + 1) / 2 * dims;
            yTemp = (tan(atan(z / y)) + 1) / 2 * dims;
            imageSelect = 5;
        }




        yPixel = yTemp > dims - 1 ? (dims - 1) : yTemp;

        if (yPixel > dims - 1)
            yPixel = dims - 1;
        if (xPixel > dims - 1)
            xPixel = dims - 1;

        uchar3 value;
        if (imageSelect == 1)
            value = front[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 2)
            value = back[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 3)
            value = top[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 4)
            value = bottom[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 5)
            value = left[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 6)
            value = right[int(yPixel) * dims + int(xPixel)];

        dst[dst_y * cols + dst_x] = value;
    }
}
''', 'cube2equi')

def div_up(a, b):
    return (a + b - 1) // b


def allocate_gpu_memory(dims):
    front_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    back_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    top_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    bottom_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    left_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    right_gpu = cp.empty((dims, dims, 3), dtype=cp.uint8)
    dst_gpu = cp.empty((dims * 2, dims * 4, 3), dtype=cp.uint8)

    return front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu


def cube2equi_cuda(front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu, dims):
    rows, cols = dims * 2, dims * 4

    block = (32, 32)
    grid = (div_up(cols, block[0]), div_up(rows, block[1]))

    cube2equi_kernel(grid, block, (front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu, rows, cols, dims))

    return cp.asnumpy(dst_gpu)
