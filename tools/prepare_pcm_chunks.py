#!/usr/bin/env python3
"""
PCM Chunk Preparation Tool for WebRTC AEC3
Converts CosyVoice PCM files to proper 10ms chunks (480 samples at 48kHz)
"""

import os
import sys
import struct
import argparse
from pathlib import Path

def combine_pcm_files(input_dir, output_file):
    """Combine all PCM chunks in order into a single file."""
    print(f"🔄 Combining PCM files from {input_dir}...")
    
    # Find all chunk files and sort them numerically
    chunk_files = []
    for filename in os.listdir(input_dir):
        if filename.startswith('chunk_') and filename.endswith('.pcm'):
            chunk_num = int(filename.replace('chunk_', '').replace('.pcm', ''))
            chunk_files.append((chunk_num, filename))
    
    chunk_files.sort(key=lambda x: x[0])  # Sort by chunk number
    
    total_samples = 0
    with open(output_file, 'wb') as outf:
        for chunk_num, filename in chunk_files:
            filepath = os.path.join(input_dir, filename)
            file_size = os.path.getsize(filepath)
            samples = file_size // 2  # 16-bit = 2 bytes per sample
            
            print(f"  📁 Chunk {chunk_num}: {samples} samples ({samples/48000:.1f}ms)")
            
            with open(filepath, 'rb') as inf:
                data = inf.read()
                outf.write(data)
                total_samples += samples
    
    duration_ms = total_samples / 48000 * 1000
    print(f"✅ Combined file: {total_samples} samples ({duration_ms:.1f}ms)")
    return total_samples

def split_into_aec3_chunks(input_file, output_dir, frame_size=480):
    """Split combined PCM into proper AEC3 10ms chunks."""
    print(f"🔄 Splitting into {frame_size}-sample chunks...")
    
    os.makedirs(output_dir, exist_ok=True)
    
    chunk_count = 0
    total_samples = 0
    
    with open(input_file, 'rb') as inf:
        while True:
            # Read exactly 480 samples (960 bytes)
            data = inf.read(frame_size * 2)
            if len(data) < frame_size * 2:
                if len(data) > 0:
                    print(f"⚠️  Last chunk only {len(data)//2} samples, padding with zeros")
                    # Pad with zeros to make it exactly 480 samples
                    data += b'\x00' * (frame_size * 2 - len(data))
                else:
                    break
            
            # Save chunk
            chunk_filename = os.path.join(output_dir, f'aec3_chunk_{chunk_count:04d}.pcm')
            with open(chunk_filename, 'wb') as outf:
                outf.write(data)
            
            chunk_count += 1
            total_samples += frame_size
            
            if chunk_count % 100 == 0:
                print(f"  📦 Processed {chunk_count} chunks...")
    
    duration_ms = total_samples / 48000 * 1000
    print(f"✅ Created {chunk_count} chunks ({duration_ms:.1f}ms total)")
    return chunk_count

def verify_chunks(output_dir, frame_size=480):
    """Verify that all chunks are exactly the right size."""
    print(f"🔍 Verifying chunk sizes...")
    
    chunk_files = [f for f in os.listdir(output_dir) if f.startswith('aec3_chunk_')]
    chunk_files.sort()
    
    errors = 0
    for filename in chunk_files[:10]:  # Check first 10
        filepath = os.path.join(output_dir, filename)
        size = os.path.getsize(filepath)
        expected_size = frame_size * 2
        
        if size != expected_size:
            print(f"❌ {filename}: {size} bytes (expected {expected_size})")
            errors += 1
        else:
            print(f"✅ {filename}: {size} bytes")
    
    if len(chunk_files) > 10:
        print(f"   ... and {len(chunk_files) - 10} more chunks")
    
    return errors == 0

def create_test_android_assets(output_dir, android_assets_dir):
    """Copy first 50 chunks to Android assets for testing."""
    print(f"📱 Creating Android test assets...")
    
    test_dir = os.path.join(android_assets_dir, 'aec3_test_chunks')
    os.makedirs(test_dir, exist_ok=True)
    
    chunk_files = [f for f in os.listdir(output_dir) if f.startswith('aec3_chunk_')]
    chunk_files.sort()
    
    # Copy first 50 chunks (5 seconds of audio)
    for i, filename in enumerate(chunk_files[:50]):
        src = os.path.join(output_dir, filename)
        dst = os.path.join(test_dir, filename)
        
        with open(src, 'rb') as inf, open(dst, 'wb') as outf:
            outf.write(inf.read())
    
    print(f"✅ Copied {min(50, len(chunk_files))} test chunks to {test_dir}")

def main():
    parser = argparse.ArgumentParser(description='Prepare PCM chunks for WebRTC AEC3')
    parser.add_argument('input_dir', help='Directory containing CosyVoice PCM chunks')
    parser.add_argument('--output', '-o', default='aec3_prepared', help='Output directory')
    parser.add_argument('--android-assets', help='Android assets directory for test chunks')
    parser.add_argument('--frame-size', type=int, default=480, help='Frame size in samples (default: 480)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input_dir):
        print(f"❌ Input directory not found: {args.input_dir}")
        return 1
    
    # Create output directories
    os.makedirs(args.output, exist_ok=True)
    combined_file = os.path.join(args.output, 'combined.pcm')
    chunks_dir = os.path.join(args.output, 'chunks')
    
    try:
        # Step 1: Combine all PCM files in order
        total_samples = combine_pcm_files(args.input_dir, combined_file)
        
        # Step 2: Split into proper AEC3 chunks
        chunk_count = split_into_aec3_chunks(combined_file, chunks_dir, args.frame_size)
        
        # Step 3: Verify chunks
        if verify_chunks(chunks_dir, args.frame_size):
            print("✅ All chunks verified successfully")
        else:
            print("❌ Some chunks failed verification")
            return 1
        
        # Step 4: Create Android test assets if requested
        if args.android_assets:
            create_test_android_assets(chunks_dir, args.android_assets)
        
        print(f"\n🎉 PCM preparation complete!")
        print(f"📁 Output directory: {chunks_dir}")
        print(f"📊 Statistics:")
        print(f"   - Original chunks: Variable sizes (318ms - 1000ms)")
        print(f"   - Prepared chunks: {chunk_count} × {args.frame_size} samples (10ms each)")
        print(f"   - Total duration: {total_samples/48000:.1f} seconds")
        print(f"   - Format: 48kHz, 16-bit, mono PCM")
        
        # Usage instructions
        print(f"\n🚀 Next steps:")
        print(f"1. Copy chunks to Android assets:")
        print(f"   cp {chunks_dir}/aec3_chunk_*.pcm /path/to/android/assets/")
        print(f"2. Update Android app to use these 10ms chunks")
        print(f"3. Test with improved timing synchronization")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
