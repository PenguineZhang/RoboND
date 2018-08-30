# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter


# PassThrough filter


# RANSAC plane segmentation


# Extract inliers

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers


# Save pcd for tabletop objects

<<<<<<< HEAD

=======
# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename_in = 'extracted_inliers.pcd'

extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename_out = 'extracted_outliers.pcd'

# Save pcd for table
pcl.save(extracted_inliers, filename_in)
pcl.save(extracted_outliers, filename_out)
print('success!')
>>>>>>> a9ce4b9ef214ad299f7141e6dbc2e1709ef5d2af
