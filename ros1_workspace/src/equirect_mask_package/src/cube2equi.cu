extern "C" __global__
void cube2equi(const uchar3* posy, const uchar3* negx, const uchar3* posx,
               const uchar3* negz, const uchar3* negy, const uchar3* posz,
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

        if (yy == -1)
        {
            xPixel = (((-1.0 * tan(atan(x / y)) + 1.0) / 2.0) * dims);
            yTemp = (((-1.0 * tan(atan(z / y)) + 1.0) / 2.0) * (dims - 1.0));
            imageSelect = 1;
        }
        else if (xx == 1)
        {
            xPixel = (((tan(atan(y / x)) + 1.0) / 2.0) * dims);
            yTemp = (((tan(atan(z / x)) + 1.0) / 2.0) * dims);
            imageSelect = 2;
        }
        else if (yy == 1)
        {
            xPixel = (((-1 * tan(atan(x / y)) + 1.0) / 2.0) * dims);
            yTemp = (((tan(atan(z / y)) + 1.0) / 2.0) * (dims - 1));
            imageSelect = 3;
        }
        else if (xx == -1) {
            xPixel = (((tan(atan(y / x)) + 1.0) / 2.0) * dims);
            yTemp = (((-1 * tan(atan(z / x)) + 1.0) / 2.0) * (dims - 1));
            imageSelect = 4;
        }
        else if (zz == 1)
        {
            xPixel = (((tan(atan(y / z)) + 1.0) / 2.0) * dims);
            yTemp = (((-1 * tan(atan(x / z)) + 1.0) / 2.0) * (dims - 1));
            imageSelect = 5;
        }
        else if (zz == -1)
        {
            xPixel = (((-1 * tan(atan(y / z)) + 1.0) / 2.0) * dims);
            yTemp = (((-1 * tan(atan(x / z)) + 1.0) / 2.0) * (dims - 1));
            imageSelect = 6;
        }

        yPixel = yTemp > dims - 1 ? (dims - 1) : yTemp;

        if (yPixel > dims - 1)
            yPixel = dims - 1;
        if (xPixel > dims - 1)
            xPixel = dims - 1;

        uchar3 value;
        if (imageSelect == 1)
            value = posy[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 2)
            value = posx[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 3)
            value = negy[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 4)
            value = negx[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 5)
            value = negz[int(yPixel) * dims + int(xPixel)];
        else if (imageSelect == 6)
            value = posz[int(yPixel) * dims + int(xPixel)];

        dst[dst_y * cols + dst_x] = value;
    }
}