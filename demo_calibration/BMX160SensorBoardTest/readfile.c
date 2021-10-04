

#include <stdio.h>
#include <stdint.h>
#include <math.h>

int main(int argc,char**argv){
    FILE* f;
    double magX,magY,magZ,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ;
    double magSX=0.0,magSY=0.0,magSZ=0.0,GyroSX=0.0,GyroSY=0.0,GyroSZ=0.0,AccelSX=0.0,AccelSY=0.0,AccelSZ=0.0;
    double magAvgX=0.0,magAvgY=0.0,magAvgZ=0.0,GyroAvgX=0.0,GyroAvgY=0.0,GyroAvgZ=0.0,AccelAvgX=0.0,AccelAvgY=0.0,AccelAvgZ=0.0;
    int timeDiff;
    double version;
    uint32_t crc1,crc2,crc3;
    int cnt = 0;
    int rc;

	if (argc < 2) {
		fprintf (stderr,"Usage: %s <file name>\n",argv[0]);
		return 1;
	}

	f = fopen(argv[1],"r");

    if (!f) {
		fprintf (stderr,"Error: Cannot open file %s	\n",argv[1]);
		return 2;
    }

    rc = fscanf(f,"%i %lf %X %X %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    		&timeDiff,&version,&crc1,&crc2,&magX,&magY,&magZ,&GyroX,&GyroY,&GyroZ,&AccelX,&AccelY,&AccelZ);
    while (rc == 13) {
    	magSX += magX;
		magSY += magY ;
		magSZ += magZ ;
		GyroSX += GyroX ;
		GyroSY += GyroY ;
		GyroSZ += GyroZ ;
		AccelSX += AccelX ;
		AccelSY += AccelY ;
		AccelSZ += AccelZ ;
		cnt++;
    rc = fscanf(f,"%i %lf %X %X %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    		&timeDiff,&version,&crc1,&crc2,&magX,&magY,&magZ,&GyroX,&GyroY,&GyroZ,&AccelX,&AccelY,&AccelZ);
    }

    fclose(f);

    printf ("rc = %d\n",rc);
    printf("cnt = %d\n",cnt);

    magAvgX = (magSX / (double)cnt);
    magAvgY = (magSY / (double)cnt);
    magAvgZ = (magSZ / (double)cnt);
    GyroAvgX = (GyroSX / (double)cnt);
    GyroAvgY = (GyroSY / (double)cnt);
    GyroAvgZ = (GyroSZ / (double)cnt);
    AccelAvgX = (AccelSX / (double)cnt);
    AccelAvgY = (AccelSY / (double)cnt);
    AccelAvgZ = (AccelSZ / (double)cnt);

	printf("Avg magX = %f\n",magAvgX);
	printf("Avg magY = %f\n",magAvgY);
	printf("Avg magZ = %f\n",magAvgZ);
	printf("Avg GyroX = %f\n",GyroAvgX);
	printf("Avg GyroY = %f\n",GyroAvgY);
	printf("Avg GyroZ = %f\n",GyroAvgZ);
	printf("Avg AccelX = %f\n",AccelAvgX);
	printf("Avg AccelY = %f\n",AccelAvgY);
	printf("Avg AccelZ = %f\n",AccelAvgZ);

	printf("Avg AccelXFactor = %f\n",1.0/AccelAvgX);
	printf("Avg AccelYFactor = %f\n",1.0/AccelAvgY);
	printf("Avg AccelZFactor = %f\n",1.0/AccelAvgZ);

	// Second round! Calculate the standard deviation
	f = fopen(argv[1],"r");

    if (!f) {
		fprintf (stderr,"Error: Cannot open file %s	\n",argv[1]);
		return 3;
    }

	cnt = 0;
	magSX = 0;
	magSY = 0;
	magSZ = 0;
	GyroSX = 0;
	GyroSY = 0;
	GyroSZ = 0;
	AccelSX = 0;
	AccelSY = 0;
	AccelSZ = 0;


    rc = fscanf(f,"%i %lf %X %X %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    		&timeDiff,&version,&crc1,&crc2,&magX,&magY,&magZ,&GyroX,&GyroY,&GyroZ,&AccelX,&AccelY,&AccelZ);
    while (rc == 13) {
    	magSX += (magX - magAvgX) * (magX - magAvgX);
		magSY += (magY - magAvgY) * (magY - magAvgY);
		magSZ += (magZ - magAvgZ) * (magZ - magAvgZ);
		GyroSX += (GyroX - GyroAvgX) * (GyroX - GyroAvgX);
		GyroSY += (GyroY - GyroAvgY) * (GyroY - GyroAvgY);
		GyroSZ += (GyroZ - GyroAvgZ) * (GyroZ - GyroAvgZ);
		AccelSX += (AccelX - AccelAvgX) * (AccelX - AccelAvgX);
		AccelSY += (AccelY - AccelAvgY) * (AccelY - AccelAvgY);
		AccelSZ += (AccelZ - AccelAvgZ) * (AccelZ - AccelAvgZ);
		cnt++;
    rc = fscanf(f,"%i %lf %X %X %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
    		&timeDiff,&version,&crc1,&crc2,&magX,&magY,&magZ,&GyroX,&GyroY,&GyroZ,&AccelX,&AccelY,&AccelZ);
    }

    fclose(f);

    printf("cnt = %d\n",cnt);
	printf("Std. Dev. magX   = %10.6f Variance magX   = %10.6f\n",sqrt(magSX/cnt),(magSX/cnt));
	printf("Std. Dev. magY   = %10.6f Variance magY   = %10.6f\n",sqrt(magSY/cnt),(magSY/cnt));
	printf("Std. Dev. magZ   = %10.6f Variance magZ   = %10.6f\n",sqrt(magSZ/cnt),(magSZ/cnt));
	printf("Std. Dev. GyroX  = %10.6f Variance GyroX  = %10.6f\n",sqrt(GyroSX/cnt),(GyroSX/cnt));
	printf("Std. Dev. GyroY  = %10.6f Variance GyroY  = %10.6f\n",sqrt(GyroSY/cnt),(GyroSY/cnt));
	printf("Std. Dev. GyroZ  = %10.6f Variance GyroZ  = %10.6f\n",sqrt(GyroSZ/cnt),(GyroSZ/cnt));
	printf("Std. Dev. AccelX = %10.6f Variance AccelX = %10.6f\n",sqrt(AccelSX/cnt),(AccelSX/cnt));
	printf("Std. Dev. AccelY = %10.6f Variance AccelY = %10.6f\n",sqrt(AccelSY/cnt),(AccelSY/cnt));
	printf("Std. Dev. AccelZ = %10.6f Variance AccelZ = %10.6f\n",sqrt(AccelSZ/cnt),(AccelSZ/cnt));



	return 0;
}
