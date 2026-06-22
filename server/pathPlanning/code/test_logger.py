# test_logger.py
import os
import logging
from datetime import datetime

def setup_file_logger() -> None:
    """Redirects all logging.debug outputs from the core code into a raw data file."""
    # 1. Target the new 'dataOutput' directory inside the current folder
    output_dir = "dataOutput"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # 2. Generate a unique filename without any file extension/type extension
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(output_dir, f"test_run_{timestamp}")
    
    # 3. Access root logger framework
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    
    # 4. Create a dedicated file handler targeting our path configuration
    file_handler = logging.FileHandler(file_path, mode="w", encoding="utf-8")
    file_handler.setFormatter(logging.Formatter("%(message)s"))
    
    # 5. Connect the stream handler pipeline
    root_logger.addHandler(file_handler)
    print(f"[LOGGER] Diverting terminal data to raw file: {file_path}")
