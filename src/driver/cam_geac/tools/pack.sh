#!/bin/bash
app_name=cameraPlatform
app_dir=/usr/local/camera_demo_geac
cam_geac_dir=./cam_geac/usr/local/camera_demo_geac
camera_demo_geac_dir=./camera_demo_geac/usr/local/camera_demo_geac
com_dir=../../cam_geac

Usage()
{
        echo -e "Usage:\n" \
                " ./pack.sh [option]\n\n\n" \
                "Options:\n" \
                " -f            configuration folder";
}

CopyCfg()
{
        cp -av $1/* $camera_demo_geac_dir/cfg
}

## delete historical records
test -d $camera_demo_geac_dir && rm -rf $camera_demo_geac_dir

## build deb package file structure
for dir in demo lib tools
do
    mkdir -p $camera_demo_geac_dir/$dir
done


## parse command line params
while true; do
        case "$1" in
                -h|--help) Usage; exit 0 ;;
                -f|--file) CopyCfg $2; shift 2;;
                "") shift ; break ;;
                *) echo "Internal error" >&2; exit 1 ;;
        esac
done


platform_name="arm64"

## app version number
app_version_file=./version
major=`sed -n  '/MAJOR/p' $app_version_file | sed  -r  's#.*=(.*)#\1#g'`
minor=`sed -n  '/MINOR/p' $app_version_file | sed  -r  's#.*=(.*)#\1#g'`
build=`sed -n  '/BUILD/p' $app_version_file | sed  -r  's#.*=(.*)#\1#g'`
version_number="$major.$minor.$build"

## app packet date
build_date=build"$(date +"%Y%m%d")"


## install cam_geac to deb package
cp -av  ${com_dir}/demo \
        ${com_dir}/lib \
        ${com_dir}/case_rb \
        ${com_dir}/rb_camera.sh \
        ${com_dir}/cfg \
        $camera_demo_geac_dir

cp -av  ${com_dir}/tools/serdes_tool.sh \
        $camera_demo_geac_dir/tools

cp -av  ${com_dir}/tools/.jq \
        $camera_demo_geac_dir/tools
        
# 要添加的内容
new_line="cd /usr/local/camera_demo_geac"

# 使用 sed 在第二行后插入内容
sed -i "2a $new_line" "$camera_demo_geac_dir/rb_camera.sh"

## update DEBIAN control file
control_file=camera_demo_geac/DEBIAN/control
echo "Package: $app_name" > $control_file
echo "Version: ${version_number}" >> $control_file
echo "Section: free" >> $control_file
echo "Priority: optional" >> $control_file
echo "Installed_Size: `du -sh $camera_demo_geac_dir | awk '{print $1}'`" >> $control_file
echo "Architecture: ${platform_name}" >> $control_file
echo "Maintainer: xxx@xxx.com" >> $control_file
echo "Description: ${app_name} debian package" >> $control_file

package_name=${app_name}_v${version_number}_${build_date}


## pack
dpkg -b camera_demo_geac ${package_name}.deb
