#!/usr/bin/env python3
"""
Utility script to generate ArUco markers for printing
"""

import cv2
import cv2.aruco as aruco
import argparse
import os


def generate_markers(output_dir, start_id, end_id, marker_size, dict_type):
    """
    Generate ArUco markers and save as images
    
    Args:
        output_dir: Directory to save marker images
        start_id: Starting marker ID
        end_id: Ending marker ID
        marker_size: Size of marker image in pixels
        dict_type: ArUco dictionary type
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get ArUco dictionary
    aruco_dict_map = {
        'DICT_4X4_50': aruco.DICT_4X4_50,
        'DICT_4X4_100': aruco.DICT_4X4_100,
        'DICT_4X4_250': aruco.DICT_4X4_250,
        'DICT_5X5_50': aruco.DICT_5X5_50,
        'DICT_5X5_100': aruco.DICT_5X5_100,
        'DICT_6X6_50': aruco.DICT_6X6_50,
        'DICT_6X6_100': aruco.DICT_6X6_100,
    }
    
    if dict_type not in aruco_dict_map:
        print(f"Unknown dictionary type: {dict_type}")
        print(f"Available types: {list(aruco_dict_map.keys())}")
        return
    
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_map[dict_type])
    
    print(f"Generating markers {start_id} to {end_id}...")
    print(f"Dictionary: {dict_type}")
    print(f"Marker size: {marker_size}x{marker_size} pixels")
    print(f"Output directory: {output_dir}")
    print()
    
    for marker_id in range(start_id, end_id + 1):
        # Generate marker image
        marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        
        # Add white border for better detection
        border_size = int(marker_size * 0.2)
        bordered_image = cv2.copyMakeBorder(
            marker_image,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=[255, 255, 255]
        )
        
        # Save image
        output_path = os.path.join(output_dir, f'aruco_marker_{marker_id}.png')
        cv2.imwrite(output_path, bordered_image)
        print(f"Generated marker ID {marker_id}: {output_path}")
    
    print()
    print("=" * 60)
    print("PRINTING INSTRUCTIONS:")
    print("=" * 60)
    print("1. Print markers on white paper (letter or A4 size)")
    print("2. Ensure printer is set to 100% scale (no scaling)")
    print("3. Use high quality print settings")
    print("4. Measure printed markers - they should be exactly 15cm x 15cm")
    print("   (or whatever size you specified in config)")
    print("5. Mount on rigid backing (cardboard, foam board)")
    print("6. Place markers flat on ground or mounting surface")
    print("7. Avoid wrinkles, shadows, and glare")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Generate ArUco markers for warehouse drone navigation'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='./aruco_markers',
        help='Output directory for marker images (default: ./aruco_markers)'
    )
    parser.add_argument(
        '--start-id',
        type=int,
        default=0,
        help='Starting marker ID (default: 0)'
    )
    parser.add_argument(
        '--end-id',
        type=int,
        default=9,
        help='Ending marker ID (default: 9)'
    )
    parser.add_argument(
        '--marker-size',
        type=int,
        default=400,
        help='Marker size in pixels (default: 400)'
    )
    parser.add_argument(
        '--dict-type',
        type=str,
        default='DICT_4X4_50',
        choices=['DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 
                 'DICT_5X5_50', 'DICT_5X5_100', 
                 'DICT_6X6_50', 'DICT_6X6_100'],
        help='ArUco dictionary type (default: DICT_4X4_50)'
    )
    
    args = parser.parse_args()
    
    generate_markers(
        args.output_dir,
        args.start_id,
        args.end_id,
        args.marker_size,
        args.dict_type
    )


if __name__ == '__main__':
    main()
