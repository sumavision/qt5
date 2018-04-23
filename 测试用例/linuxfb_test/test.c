#include <sys/types.h>
        #include <sys/stat.h>
        #include <sys/ioctl.h>
        #include <sys/mman.h>
        #include <fcntl.h>
        #include <stdio.h>
        #include <string.h>
        #include <linux/fb.h>
         
         
   
        #define    WIN2    0
     
        #define    WIN_MAX    1
         
        const char * fb_file_path[ WIN_MAX ] = {
        
        /* WIN2 */"/dev/fb0"
        };
        const int fb_x[ WIN_MAX ] = {
        
        /* WIN2 */300,
       
        };
        const int fb_color[ WIN_MAX ] = {
        
        /* WIN2 */0x1000FF00,
       
        };
         
        void draw_framebuffer( int win, int fd, int xres, int yres )
        {
            int i, j;
            int *p;
            int color;
             
            p = mmap( NULL, xres*yres*4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0 );
            if( !p ){
                printf( "mmap failed\n" );
                return;
            }
         
            /* draw default color with alpha : all pass through*/
            for( j = 0; j < yres; j++ ){
                for( i = 0; i < xres ; i++ ){
                    *( p + j * xres + i ) = 0x00FFFFFF;
                }
            }
         
            /* draw color */
            for( j = 0; j < yres; j++ ){
                for( i = 0; i < fb_x[win] ; i++ ){
                    *( p + j * xres + i ) = fb_color[win];
                }
            }
         
            munmap( p, xres*yres*4 );
        }
         
        int main( int argc, char *argv[] )
        {
            int i;
            int ret;
            struct fb_var_screeninfo var;
            int    blank;
            int fd[ WIN_MAX ];
         
            /* initial */
            memset( fd, -1, sizeof( fd ) );
         
            /* open framebuffer */
            for( i=0; i<WIN_MAX; i++ ){
                fd[i] = open( fb_file_path[i], O_RDWR );
                if( fd[i] < 0 ){
                    printf( "open %s failed\n", fb_file_path[i] );
                    goto end;
                }
            }
            printf( "open framebuffer ok\n" );
         
            /* set screen info */
            ret = ioctl( fd[WIN2], FBIOGET_VSCREENINFO, &var );
            if( ret < 0 ){
                printf( "ioctl %s FBIOGET_VSCREENINFO failed\n", fb_file_path[WIN2] );
                goto end;
            }
         
            var.activate = FB_ACTIVATE_FORCE;
            var.yres_virtual = var.yres;
             
            for( i=0; i<WIN_MAX; i++ ){
                ret = ioctl( fd[i], FBIOPUT_VSCREENINFO, &var );
                if( ret < 0 ){
                    printf( "ioctl %s FBIOPUT_VSCREENINFO failed\n", fb_file_path[i] );
                    goto end;
                }
            }
            printf( "set screeninfo ok\n" );
         
            /* draw some color */
            for( i=0; i<WIN_MAX; i++ ){
                draw_framebuffer( i, fd[i], var.xres, var.yres );
            }
            printf( "draw color ok\n" );
         
            /* show window */
            blank = FB_BLANK_UNBLANK;
         
            for( i=0; i<WIN_MAX; i++ ){
                ret = ioctl( fd[i], FBIOBLANK, blank );
                if( ret < 0 ){
                    printf( "ioctl %s FBIOBLANK failed\n", fb_file_path[i] );
                    goto end;
                }
            }
            printf( "show window ok\n" );
         
            /* wait input */    
            getchar();
        end:
            return 0;
        }
