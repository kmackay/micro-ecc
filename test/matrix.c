#include <stdio.h>

// int params[] = {
//     1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// unsigned size = 128;

int params[] = {
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned size = 160;

int main()
{
    int matrix[size][size];
    
    unsigned i, j;
    for(i=0; i<size; ++i)
    {
        for(j=0; j<size; ++j)
        {
            if(i == 0)
            {
                matrix[i][j] = params[j];
            }
            else if(j == 0)
            {
                matrix[i][j] = params[j] * matrix[i-1][size-1];
            }
            else
            {
                matrix[i][j] = matrix[i-1][j-1] + params[j] * matrix[i-1][size-1];
            }
            
            printf("%d ", matrix[i][j]);
        }
        printf("\n");
    }
    
    int max = 0;
    for(j=0; j<size; ++j)
    {
        int sum = 0;
        for(i=0; i<size; ++i)
        {
            sum += matrix[i][j];
        }
        if(sum > max)
        {
            max = sum;
        }
    }
    printf("w = %d\n", max);
    
    int af[max][size];
    
    int k;
    for(k=0; k<max; ++k)
    {
        for(j=0; j<size; ++j)
        {
            i = 0;
            while(i < size && matrix[i][j] == 0)
            {
                ++i;
            }
            if(i < size)
            {
                af[k][j] = size + i;
                --matrix[i][j];
            }
            else
            {
                af[k][j] = 0;
            }
            
            printf("%d-%d ", af[k][j] / 32, af[k][j] % 32);
            if(j % 32 == 31)
            {
                printf("\n");
            }
        }
        printf("\n");
    }
    
    return 0;
}
