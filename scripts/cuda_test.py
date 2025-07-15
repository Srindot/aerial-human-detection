import torch

# checks for cuda on the device

def main():
    print("Checking CUDA device availability...\n")

    if not torch.cuda.is_available():
        print("❌ CUDA is NOT available on this system.")
        return

    num_devices = torch.cuda.device_count()
    print(f"✅ {num_devices} CUDA device(s) found.\n")

    for i in range(num_devices):
        device = torch.device(f'cuda:{i}')
        device_name = torch.cuda.get_device_name(i)
        total_mem = torch.cuda.get_device_properties(i).total_memory / (1024 ** 3)
        print(f"Device {i}: {device_name}")
        print(f"  Total Memory: {total_mem:.2f} GB")
        print(f"  Current Device: {torch.cuda.current_device() == i}")
        print("-" * 40)

if __name__ == "__main__":
    main()
