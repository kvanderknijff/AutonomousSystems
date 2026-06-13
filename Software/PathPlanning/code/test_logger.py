import os
import logging
from datetime import datetime

def setup_file_logger() -> None:
    """Redirects all logging.debug outputs from the core code into a text file."""
    # 1. Create the 'testData' directory if it doesn't exist yet
    output_dir = "testData"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # 2. Generate a unique filename using a timestamp (e.g., test_20260614_123000.txt)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Tip: Remove the '.txt' below if you want files without any extension/type!
    file_path = os.path.join(output_dir, f"test_run_{timestamp}")
    
    # 3. Get the root logger used by path_planner.py
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    
    # 4. Create a file handler that writes to our new file path
    file_handler = logging.FileHandler(file_path, mode="w", encoding="utf-8")
    file_handler.setFormatter(logging.Formatter("%(message)s"))
    
    # 5. Attach the file handler to the logger
    root_logger.addHandler(file_handler)
    print(f"[LOGGER] Diverting terminal data to file: {file_path}")
