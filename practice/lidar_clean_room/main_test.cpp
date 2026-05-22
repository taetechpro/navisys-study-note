#include "my_lidar_segmenter.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bin> <output.ply>\n";
        return 1;
    }
    const std::string bin_path = argv[1];
    const std::string ply_path = argv[2];

    try {
        // 1. KITTI .bin 로드
        const auto raw_points = load_kitti_velodyne_bin(bin_path);
        std::cout << "Loaded " << raw_points.size() << " raw points\n";

        // 2. ROI + finite 필터
        LidarSegmentationOptions opt;
        const auto roi_points = filter_roi(raw_points, opt);
        std::cout << "After ROI/finite: " << roi_points.size() << " points\n";

        // 3. Floor 분리
        Plane floor_plane;
        std::vector<std::size_t> floor_inliers;
        const bool floor_ok = segment_floor(roi_points, opt,
                                            floor_plane, floor_inliers);
        std::cout << "Floor: " << (floor_ok ? "OK" : "FAIL")
                  << ", inliers = " << floor_inliers.size() << "\n";
        if (floor_ok) {
            std::cout << "  plane: " << floor_plane.a << "x + "
                      << floor_plane.b << "y + "
                      << floor_plane.c << "z + "
                      << floor_plane.d << " = 0\n";
        }

        // 4. Walls 분리 (그리디 피링)
        std::vector<std::size_t> left_inliers, right_inliers, front_inliers;
        segment_all_walls(roi_points, opt, floor_inliers,
                          left_inliers, right_inliers, front_inliers);
        std::cout << "Walls -- left: "  << left_inliers.size()
                  << ", right: "        << right_inliers.size()
                  << ", front: "        << front_inliers.size() << "\n";

        // 5. SegmentedCloud 라벨링
        SegmentedCloud cloud;
        cloud.points = roi_points;
        cloud.labels.assign(roi_points.size(), SegmentLabel::Other);
        for (auto i : floor_inliers)  cloud.labels[i] = SegmentLabel::Floor;
        for (auto i : left_inliers)   cloud.labels[i] = SegmentLabel::LeftWall;
        for (auto i : right_inliers)  cloud.labels[i] = SegmentLabel::RightWall;
        for (auto i : front_inliers)  cloud.labels[i] = SegmentLabel::FrontWall;

        // 6. counts 출력
        const auto counts = cloud.counts();
        std::cout << "Counts: other=" << counts.other
                  << " floor="        << counts.floor
                  << " left="         << counts.left_wall
                  << " right="        << counts.right_wall
                  << " front="        << counts.front_wall << "\n";

        // 7. PLY 저장
        write_segmented_ply(ply_path, cloud);
        std::cout << "Wrote " << ply_path << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
