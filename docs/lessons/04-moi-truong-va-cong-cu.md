# Môi trường và công cụ

## 1. Git trong WSL không đọc được worktree tạo bởi Git Windows (#5)

- **Triệu chứng:** trong một worktree, các script WSL dùng chung một thư mục build cho mọi nhánh,
  và `lint.sh` lặng lẽ bỏ qua clang-format.
- **Nguyên nhân:** file `.git` của worktree chứa `gitdir: C:/...`; git trong WSL không mở được, nên
  không xác định được nhánh và danh sách file thay đổi.
- **Xử lý:** `tools/wsl/git_env.sh` đổi đường dẫn sang `/mnt/c/...` và đặt `GIT_DIR`,
  `GIT_WORK_TREE`.
- **Bài học:** công cụ mà "lặng lẽ không làm gì" nguy hiểm hơn công cụ báo lỗi; script nên in ra
  nó đã kiểm tra những gì.

## 2. Workspace colcon trỏ vào thư mục nguồn đã xoá (#30)

- **Triệu chứng:** `CMake Error: The source directory ".../cobot-wt/mvp-results/ros2_node" does not
  exist`.
- **Nguyên nhân:** cache CMake của workspace dùng chung ghi đường dẫn nguồn của lần build trước
  (một worktree đã xoá). Biến thể khác của cùng lỗi: build từ worktree A rồi chạy E2E từ worktree B
  sẽ chạy code của A.
- **Xử lý:** `sim.sh build` đọc `CMAKE_HOME_DIRECTORY` trong cache; nguồn nằm ngoài repo hiện tại
  thì xoá `build/` và `install/` rồi build lại. Mỗi worktree dùng `COBOT_COLCON_WS` riêng. Log
  colcon ghi vào workspace thay vì thư mục đang đứng.
- **Bài học:** cache build phải gắn với đường dẫn nguồn, hoặc tự kiểm tra đường dẫn đó.

## 3. Hai dự án ROS 2 trên cùng WSL dùng chung domain 0 (#30)

- **Rủi ro:** chạy mô phỏng của dự án này cùng lúc với dự án SLAM thì `/clock`, `/joint_states` và
  topic của Gazebo trộn vào nhau, không có lỗi nào báo ra.
- **Xử lý:** `source tools/wsl/env.sh` đặt `ROS_DOMAIN_ID=42` (ghi đè giá trị 0 trong `~/.bashrc`)
  và `GZ_PARTITION=cobot`; đổi được bằng `COBOT_ROS_DOMAIN_ID`, `COBOT_GZ_PARTITION`. Không sửa
  `~/.bashrc` để không ảnh hưởng dự án kia.
- **Bài học:** cách ly ở mức dự án bằng biến môi trường là đủ, không cần tách hệ điều hành, nhưng
  phải đặt trong script của dự án để không ai quên.

## 4. Xoá worktree từ Windows thất bại; đường dẫn bị Git Bash đổi ngầm

- **Triệu chứng:** `git worktree remove` báo "Function not implemented"; lệnh `wsl rm -rf
  /mnt/c/...` chạy từ Git Bash không xoá gì mà không báo lỗi.
- **Nguyên nhân:** colcon tạo symlink kiểu Linux trong worktree, Windows không xoá được. Git Bash
  tự đổi đối số dạng `/mnt/c/...` thành `C:/Program Files/Git/mnt/c/...` trước khi chuyển cho WSL.
- **Xử lý:** xoá thư mục bằng `rm -rf` bên trong WSL với `MSYS_NO_PATHCONV=1` khi gọi từ Git Bash,
  rồi `git worktree prune`.
- **Bài học:** kiểm tra kết quả của lệnh xoá (thư mục còn hay không), đừng chỉ tin mã thoát.

## 5. Thư mục build cũ chiếm 25 GB, rồi lại 16 GB

- **Nguyên nhân:** mỗi nhánh có thư mục build riêng (Release, ASan, coverage) và workspace colcon
  riêng; nhánh merge xong không ai dọn. Lần dọn tay đầu tiên không sửa gốc, nên chỉ vài giờ sau
  một đợt 6 tính năng đã sinh lại 16 GB (bản ASan 2.6–3.4 GB mỗi nhánh). Các thư mục của dự án
  nằm rải rác (`~/cobot-build`, `~/.venvs/cobot`, kết quả chạy trong `~`), lẫn với thư mục của dự
  án khác.
- **Xử lý:** mọi thứ của dự án nằm dưới một thư mục `~/projects/7dof_dynamic_control/`
  (`build/<nhánh>[-asan|-cov]`, `colcon/<nhánh>`, `venv`, `out`), đường dẫn tính ở một chỗ
  (`tools/wsl/paths.sh`). Workspace colcon theo nhánh nên hết cảnh hai worktree dùng chung một
  workspace. `tools/wsl/prune.sh` xoá build và workspace của các nhánh không còn; chạy sau mỗi lần
  merge.
- **Bài học:** dọn tay một lần không phải là cách sửa; phải sửa chỗ sinh ra rác và tự động hoá
  bước dọn.

## 6. `git pull` trong WSL bị từ chối (publickey)

- **Nguyên nhân:** khoá SSH chỉ có ở phía Windows.
- **Xử lý:** pull từ Windows (cùng thư mục), hoặc trong WSL đặt
  `git config --global core.sshCommand ssh.exe` để dùng ssh và khoá của Windows.
