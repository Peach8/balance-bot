#include <stdio.h>
#include <stdlib.h>
double *movavgfilter(double* unsortedar, int arrlen, int n){
    int i;
	double *filtered = (double*)malloc(arrlen*sizeof(double));
    for(i = 0; i < n; i++){
		filtered[i] = unsortedar[i];
	}
	for(i = n; i < arrlen; i++){
		double temparr[n];
		int j;
		for(j = 0; j<n; j++){
		    temparr[j] = unsortedar[i-n+j];
		    //printf("%lf\n", temparr[j]);
		}
		int k;
		double sum = 0.0;
		for (j = 0; j<n; j++){
			sum += temparr[j];
		}
        filtered[i] = sum/n;
	}
	for (i = 0; i < arrlen; i++){
		printf("%lf\n", filtered[i]);
	}
	return filtered;
}

int main(){	
	double peach[15] = {3.0, 5.0, 2.0, 6.0, 1.0, 4.0, 7.0, 15.0, 13.0, 8.0, 11.0, 14.0, 12.0, 9.0, 10.0}; 
	movavgfilter(peach,15, 4);
}












