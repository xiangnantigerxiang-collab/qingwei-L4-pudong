#include "Spline.h"
#include <math.h>

void CSpline::SplinePointSet(std::vector<XYZ_COOR_S> tSrcCoorList,
                             std::vector<XYZ_COOR_S> &tDesCoorList,
                             float tRes)
{
    int num = tSrcCoorList.size();

    if(num <= 3) return;
 
    float tmp_s = 0;
    float *x = new float[num];
    float *y = new float[num];
    float *s = new float[num];

    for(int i = 0;i < num;i++) {
        x[i] = tSrcCoorList.at(i).x_axis;
        y[i] = tSrcCoorList.at(i).y_axis;
    }

    s[0] = 0;

    for(int i = 0; i < num-1; i++) {
        double dx = x[i+1] - x[i];
	double dy = y[i+1] - y[i];

        tmp_s = tmp_s + hypot(dx, dy);
        s[i+1] = tmp_s;
    }

    //smoothed spline points
    int N = (int)(tmp_s / tRes);
    float *sd=new float[N];
    float *xd=new float[N];
    float *yd=new float[N];

    for(int i = 0; i < N-1; i++) sd[i] = i * tRes;

    sd[N-1] = tmp_s;

    //自然边界条件
    float f11x = 0.0;
    float f11y = 0.0;
    float f22x = 0.0; 
    float f22y = 0.0;
    float *mx;
    float *my;

    Spline2(s, x, num, mx, f11x, f22x);
    Spline2(s, y, num, my, f11y, f22y);

    for(int i = 0; i < N; i++) {
        Splint(s, x, mx, num, sd[i], xd[i]);
        Splint(s, y, my, num, sd[i], yd[i]);
    }

    tDesCoorList.clear();

    for(int i = 0; i < N; i++) {
        XYZ_COOR_S xyz_temp;
        xyz_temp.x_axis = xd[i];
        xyz_temp.y_axis = yd[i];
        tDesCoorList.push_back(xyz_temp);
    }

    delete sd;
    delete xd;
    delete yd;
    delete mx;
    delete my;
}

void CSpline::SplinePointSet(std::vector<XYZ_COOR_S> tSrcCoorList,
                             std::vector<XYZ_COOR_S> &tDesCoorList,
                             float tRes,
                             std::vector<XYZ_COOR_S> tSrcList,
                             std::vector<XYZ_COOR_S> &tDesList)
{
    int num = tSrcCoorList.size();

    if(num <= 3) return;
    
    float tmp_s = 0;
    float *x = new float[num];
    float *y = new float[num];
    float *s = new float[num];

    for(int i = 0; i < num; i++) {
        x[i] = tSrcCoorList.at(i).x_axis;
        y[i] = tSrcCoorList.at(i).y_axis;
    }

    s[0] = 0;

    for(int i = 0; i < num-1; i++) {
        tmp_s=tmp_s+sqrt((x[i+1]-x[i])*(x[i+1]-x[i])+(y[i+1]-y[i])*(y[i+1]-y[i]));
        s[i+1]=tmp_s;
    }

    //smoothed spline points
    int N = (int)(tmp_s/tRes);
    float *sd=new float[N];
    float *xd=new float[N];
    float *yd=new float[N];

    for(int i=0;i<N-1;i++)
        sd[i]=i*tRes;
    sd[N-1]=tmp_s;

    //自然边界条件
    float f11x=0,f11y=0,f22x=0,f22y=0;
    float *mx,*my;
    Spline2(s,x,num,mx,f11x,f22x);
    Spline2(s,y,num,my,f11y,f22y);

    for(int i=0;i<N;i++)
    {
        Splint(s,x,mx,num,sd[i],xd[i]);
        Splint(s,y,my,num,sd[i],yd[i]);
    }

    tDesCoorList.clear();
    for(int i = 0;i < N;i++){
        XYZ_COOR_S xyz_temp;
        xyz_temp.x_axis = xd[i];
        xyz_temp.y_axis = yd[i];
        tDesCoorList.push_back(xyz_temp);
    }

    for(int i=0;i< tSrcList.size();i++)
    {
        float temp_t = 0;
        XYZ_COOR_S xyz_temp;
        if(i%4 == 0)
            temp_t = s[i/4];
        else{
            temp_t = s[i/4] + sqrt((tSrcList.at(i).x_axis - tSrcCoorList.at(i/4).x_axis)*(tSrcList.at(i).x_axis - tSrcCoorList.at(i/4).x_axis)+
                                   (tSrcList.at(i).y_axis - tSrcCoorList.at(i/4).y_axis)*(tSrcList.at(i).y_axis - tSrcCoorList.at(i/4).y_axis));
        }
        if(temp_t > tmp_s)
            break;
        Splint(s,x,mx,num,temp_t,xyz_temp.x_axis);
        Splint(s,y,my,num,temp_t,xyz_temp.y_axis);
        tDesList.push_back(xyz_temp);
    }

    tDesCoorList.clear();
    for(int i = 0;i < N;i++){
        XYZ_COOR_S xyz_temp;
        xyz_temp.x_axis = xd[i];
        xyz_temp.y_axis = yd[i];
        tDesCoorList.push_back(xyz_temp);
    }

    delete sd;
    delete xd;
    delete yd;
    delete mx;
    delete my;
}

void CSpline::Splint(float *xa,float *ya,float *m,int n,float &x,float &y)
{
	int klo,khi,k;
	klo=0; khi=n-1;
	float hh,bb,aa;
 
	while(khi-klo>1)            //  二分法查找x所在区间段
	{
		k=(khi+klo)>>1;
		if(xa[k]>x)  khi=k;
		else klo=k;
	}
	hh=abs(xa[khi]-xa[klo]);
 
	aa=(xa[khi]-x)/hh;
	bb=(x-xa[klo])/hh;
 
	y=aa*ya[klo]+bb*ya[khi]+((aa*aa*aa-aa)*m[klo]+(bb*bb*bb-bb)*m[khi])*hh*hh/6.0;
 
}

void CSpline::Spline1(float *xa,float *ya,int n,float *&m,float bound1,float bound2)
{
                                        //  追赶法解方程求二阶偏导数
	float f1=bound1,f2=bound2;
 
	float *a=new float[n];                //  a:稀疏矩阵最下边一串数
	float *b=new float[n];                //  b:稀疏矩阵最中间一串数
	float *c=new float[n];                //  c:稀疏矩阵最上边一串数
	float *d=new float[n];
 
	float *f=new float[n];
 
	float *bt=new float[n];
	float *gm=new float[n];
 
	float *h=new float[n];
	m=new float[n];
 
	for(int i=0;i<n;i++)  b[i]=2;          //  中间一串数为2
	for(int i=0;i<n-1;i++)  h[i]=(xa[i+1]-xa[i]);                   // 各段步长
	for(int i=1;i<n-1;i++)  a[i]=h[i-1]/(h[i-1]+h[i]);            
	a[n-1]=1;
 
	c[0]=1;
	for(int i=1;i<n-1;i++)  c[i]=h[i]/(h[i-1]+h[i]);
 
	for(int i=0;i<n-1;i++) 
		f[i]=(ya[i+1]-ya[i])/((xa[i+1]-xa[i]));
 
	d[0]=6*(f[0]-f1)/h[0];
	d[n-1]=6*(f2-f[n-2])/h[n-2];
 
	for(int i=1;i<n-1;i++)  d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);
 
	bt[0]=c[0]/b[0];                                             //  追赶法求解方程
	for(int i=1;i<n-1;i++)  bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);
 
	gm[0]=d[0]/b[0];
	for(int i=1;i<=n-1;i++)  gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);
 
	m[n-1]=gm[n-1];
	for(int i=n-2;i>=0;i--)  m[i]=gm[i]-bt[i]*m[i+1];
 
	delete a;
	delete b;
	delete c;
	delete d;
	delete gm;
	delete bt;
	delete f;
	delete h;
}

void CSpline::Spline2(float *xa, float *ya, int n, float *&m, float bound1, float bound2)
{
    //追赶法解方程求二阶偏导数
    float f11 = bound1;
    float f22 = bound2;
    float *a = new float[n]; //a:稀疏矩阵最下边一串数
    float *b = new float[n]; //b:稀疏矩阵最中间一串数
    float *c = new float[n]; //c:稀疏矩阵最上边一串数
    float *d = new float[n];
    float *f = new float[n];
    float *bt = new float[n];
    float *gm = new float[n];
    float *h = new float[n];

    m = new float[n];
 
    for(int i = 0; i < n; i++) b[i] = 2;
    for(int i = 0; i < n-1; i++) h[i] = xa[i+1]-xa[i];
    for(int i = 1; i < n-1; i++) a[i] = h[i-1] / (h[i-1]+h[i]);

    a[n-1]=1;
    c[0]=1;

    for(int i = 1; i < n-1; i++) c[i] = h[i] / (h[i-1]+h[i]);
    for(int i = 0; i < n-1; i++) f[i] = (ya[i+1]-ya[i]) / (xa[i+1]-xa[i]);
    for(int i = 1; i < n-1; i++) d[i] = 6 * (f[i]-f[i-1]) / (h[i-1]+h[i]);
 
    d[1] = d[1]-a[1] * f11;
    d[n-2] = d[n-2] - c[n-2] * f22;
	                                               
    //追赶法求解方程
    bt[1] = c[1]/b[1];
    for(int i = 2; i < n-2; i++) bt[i] = c[i] / (b[i]-a[i]*bt[i-1]);
 
    gm[1] = d[1]/b[1];
    for(int i = 2; i <= n-2; i++) gm[i] = (d[i]-a[i]*gm[i-1]) / (b[i]-a[i]*bt[i-1]);
 
    m[n-2] = gm[n-2];
    for(int i = n-3; i >= 1; i--) m[i] = gm[i]-bt[i]*m[i+1];
 
    m[0]=f11;
    m[n-1]=f22;
 
    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;
}

