
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "c_img.h"
#include "seamcarving.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void calc_energy(struct rgb_img *im, struct rgb_img **grad){
    int i = im->height - 1; // x values
    int j = im->width - 1; // y values 
    int w = im->width;
    int rx, gx, bx, ry, gy, by, dx, dy;
    create_img(grad, im->height, im->width);
    (*grad) -> width = im->width;
    (*grad) -> height = im->height;
    uint8_t* temp = (uint8_t *)malloc( im->height * im->width);
    for (int y = 0; y <= i; y++){
        for (int x = 0; x <= j; x++){
            if(x == 0){
                rx = im->raster[y*w*3+3] - im->raster[(y+1)*w*3-3];
                gx = im->raster[y*w*3+4] - im->raster[(y+1)*w*3-2];
                bx = im->raster[y*w*3+5] - im->raster[(y+1)*w*3-1];
            }else if(x == j){
                rx = im->raster[y*w*3] - im->raster[(y+1)*w*3-6];
                gx = im->raster[y*w*3 + 1] - im->raster[(y+1)*w*3-5];
                bx = im->raster[y*w*3 + 2] - im->raster[(y+1)*w*3-4];
            }else{
                rx = im->raster[y*w*3+(x+1)*3] - im->raster[y*w*3+(x-1)*3];
                gx = im->raster[y*w*3+(x+1)*3+1] - im->raster[y*w*3+(x-1)*3+1];
                bx = im->raster[y*w*3+(x+1)*3+2] - im->raster[y*w*3+(x-1)*3+2];
            }
            if (y == 0){
                ry = im->raster[w*3+x*3] - im->raster[i*w*3+x*3];
                gy = im->raster[w*3+x*3+1] - im->raster[i*w*3+x*3+1];
                by = im->raster[w*3+x*3+2] - im->raster[i*w*3+x*3+2];
            }else if (y == i){
                ry = im->raster[(y-1)*w*3+x*3] - im->raster[x*3];
                gy = im->raster[(y-1)*w*3+x*3+1] - im->raster[x*3+1] ;
                by = im->raster[(y-1)*w*3+x*3+2] - im->raster[x*3+2] ;
            }else{
                ry = im->raster[(y-1)*w*3+x*3] - im->raster[(y+1)*w*3+x*3];
                gy = im->raster[(y-1)*w*3+x*3+1] - im->raster[(y+1)*w*3+x*3+1];
                by = im->raster[(y-1)*w*3+x*3+2] - im->raster[(y+1)*w*3+x*3+2];
                
            }
            dy = ry * ry + by * by + gy * gy;
            dx = rx * rx + bx * bx + gx * gx;
            // printf("%d  ", rx);
            // printf("%d  ", x);
            // printf("%d  \n", y);
            set_pixel(*grad, y,x,sqrt(dy + dx)/10,sqrt(dy + dx)/10,sqrt(dy + dx)/10);
        }
    }
}

void dynamic_seam(struct rgb_img *grad, double **best_arr){
    int w = grad->width;
    int h = grad->height;
    *best_arr = malloc(w*h*sizeof(double));
    for (int y = 0; y < h; y++){    
        for (int x = 0; x < w; x++){
            if (y == 0){
                *((*best_arr) + x) = get_pixel(grad, y, x, 0);
            }else if(x == 0){
                *((*best_arr)+y*w) = MIN(*((*best_arr)+(y-1)*w+x), *((*best_arr)+(y-1)*w+x+1)) + get_pixel(grad, y, x, 0);
            }else if (x != w-1){
                *((*best_arr)+y*w+x) = MIN(MIN(*((*best_arr)+(y-1)*w+x), *((*best_arr)+(y-1)*w-1+x)), MIN(*((*best_arr)+(y-1)*w+x), *((*best_arr)+(y-1)*w+x+1))) + get_pixel(grad, y, x, 0);
            }else{
                *((*best_arr)+y*w+x) = MIN(*((*best_arr)+(y-1)*w+x), *((*best_arr)+(y-1)*w-1+x)) + get_pixel(grad, y, x, 0);
            }
        }
    }
}

void recover_path(double *best, int height, int width, int**path){
    *path = malloc(height*sizeof(int));
    int best_in_y = 1215752192;
    int prev_index;
    for (int y = height-1; y >= 0; y--){  
        if (y == height-1){
            for (int x = 0; x < width; x++){
                if (*(best+y*width+x) < best_in_y){
                    best_in_y = *(best+y*width+x);
                    *((*path) + y) = x;
                    prev_index = x;
                }
            }
        }else if(prev_index == 0){
            if(*(best+y*width+prev_index) <= *(best+y*width+prev_index+1)){
                *((*path) + y) = prev_index;
            }else{
                *((*path) + y) = prev_index+1;
                prev_index = prev_index+1;
            }
        }else if(prev_index == width-1){
            if(*(best+y*width+prev_index) <= *(best+y*width+prev_index-1)){
                *((*path) + y) = prev_index;
            }else{
                *((*path) + y) = prev_index-1;
                prev_index = prev_index-1;
            }
        }else{
            if(*(best+y*width+prev_index) <= *(best+y*width+prev_index-1)){
                if(*(best+y*width+prev_index) <= *(best+y*width+prev_index+1)){
                    *((*path) + y) = prev_index;
                }else{
                    *((*path) + y) = prev_index+1;
                    prev_index = prev_index+1;
                }
            }else{
                if(*(best+y*width+prev_index-1) <= *(best+y*width+prev_index+1)){
                    *((*path) + y) = prev_index-1;
                    prev_index = prev_index-1;
                }else{
                    *((*path) + y) = prev_index+1;
                    prev_index = prev_index+1;
                }
            }
        }
    }
}

void remove_seam(struct rgb_img *src, struct rgb_img **dest, int *path){
    int w = src -> width;
    int h = src -> height;
    create_img(dest, src->height, src->width-1);
    (*dest) -> width = src->width-1;
    (*dest) -> height = src->height;
    int counter;

    for (int y = 0; y < h; y++){
        counter = 0;
        for (int x = 0; x < w; x++){
            if(x == *(path+y)){
                continue;
            }else{
                set_pixel(*dest, y, counter, get_pixel(src,y,x,0),get_pixel(src,y,x,1),get_pixel(src,y,x,2));
                counter = counter + 1;
            }
        }
    }
}

// int main(){
// //     int height = 5;
// //     int width = 6;

// //     struct rgb_img *im;
// //     create_img(&im, height, width);
// //     read_in_img(&im, "6x5.bin");
// //     // printf("%d",im->raster[1*(2+1)*3+(1+1)*3+1] - im->raster[1*(2+1)*3+(1-1)*3+1]);
    
// //     // print_grad(im);

// //     struct rgb_img *grad;
// //     calc_energy(im, &grad);
// //     print_grad(grad);
// //     double *best_arr;
// //     dynamic_seam(grad, &best_arr);
// //     for (int i = 0; i < width*height; i++){
// //         if(i % width == 0){
// //             printf("%s", "\n");
// //         }
// //         printf("%f  ", *(best_arr+i));
// //     }
// //     printf("%s", "\n");
// //     int* path;
// //     recover_path(best_arr, height, width, &path);

// //     for(int i = 0; i < height; i++){
// //         printf("%d      ", *(path+i));
// //     }
// //     struct rgb_img *new_im;
// //     remove_seam(im, &new_im, path);


// //     for (int y = 0; y < height; y++){
// //         printf("\n");
// //         for (int x = 0; x < width; x++){
// //             printf("(%d %d %d)", get_pixel(im,y,x,0),get_pixel(im,y,x,1),get_pixel(im,y,x,2));
// //         }
// //     }
// // printf("\n");
// //     for (int y = 0; y < height; y++){
// //         printf("\n");
// //         for (int x = 0; x < width-1; x++){
// //             printf("(%d %d %d)", get_pixel(new_im,y,x,0),get_pixel(new_im,y,x,1),get_pixel( new_im,y,x,2));
// //         }
// //     }

// struct rgb_img *im;
//     struct rgb_img *cur_im;
//     struct rgb_img *grad;
//     double *best;
//     int *path;

//     read_in_img(&im, "image (1).bin");
    
//     for(int i = 0; i < 50; i++){
//         printf("i = %d\n", i);
//         calc_energy(im,  &grad);
//         dynamic_seam(grad, &best);
//         recover_path(best, grad->height, grad->width, &path);
//         remove_seam(im, &cur_im, path);

        


//         destroy_image(im);
//         destroy_image(grad);
//         free(best);
//         free(path);
//         im = cur_im;
//     }
//     write_img(cur_im, "new_im.bin");
//     destroy_image(im);
// }