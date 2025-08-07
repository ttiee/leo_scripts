## `zsh` 安装和配置

`zsh` 是一个强大的 **shell**，具有许多功能和插件，可以提高工作效率。以下是 `zsh` 的安装和配置步骤。

### 1. 安装 `zsh`

```bash
sudo apt install zsh
```

### 2. 安装 `powerlevel10k` 主题

```bash
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ~/powerlevel10k
echo 'source ~/powerlevel10k/powerlevel10k.zsh-theme' >>~/.zshrc
```

### 3. `powerlevel10k` 配置

```bash
zsh # 进入zsh
p10k configure
```

### 4. 安装 `zsh-autosuggestions` 补全插件

```bash
git clone https://github.com/zsh-users/zsh-autosuggestions ~/.zsh/zsh-autosuggestions

echo "source ~/.zsh/zsh-autosuggestions/zsh-autosuggestions.zsh" >> ~/.zshrc

source ~/.zshrc
```

### 5. 安装 `zsh-syntax-highlighting` 语法高亮插件

```bash
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git

echo "source ${(q-)PWD}/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh" >> ${ZDOTDIR:-$HOME}/.zshrc
```

### 6. 将 `zsh` 切换为默认终端

```bash
chsh -s $(which zsh)
```

### 7. 环境变量配置

将 `~/.bashrc` 中的 `export xxxxx` 全部复制添加到 `~/.zshrc` 中。如 `~/.bashrc` 中有以下内容：

```bash
source /opt/ros/kinetic/setup.bash
source ~/dashgo_ws/devel/setup.bash
source ~/moveit_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.31.200:11311
```

将 `.bash` 改为 `.zsh`，并添加到 `~/.zshrc` 中：

```bash
source /opt/ros/kinetic/setup.zsh
source ~/dashgo_ws/devel/setup.zsh
source ~/moveit_ws/devel/setup.zsh
export ROS_MASTER_URI=http://192.168.31.200:11311
```
