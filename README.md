# FirstRustGame

## Prerequisites

You need at least Rust version 1.27.0

On OSX you need to install:

```sh
brew install sdl2
```

If you don't have brew please run the following command to install:
```
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

## Build & Run

```
cargo run
```

## Game

<img width="912" alt="screen shot 2018-07-07 at 18 03 04" src="https://user-images.githubusercontent.com/223045/42412193-390bd758-8210-11e8-8cf3-45792e69aa27.png">

# About

This Project uses the OpenGL Library "SDL2" for Graphics functionality (Creating Windows, Rendering, etc).

The Data Structures used for the game are as follows:
```
pub struct Game {
  .......
}
```

The Game struct contains all the objects needed to run the game such as the player Actor, the walls (which are stored in a vector), etc

```
struct Assets {
  ..........
}
```

The Assets struct contains all the assets used in the game (mainly images, fonts and sound so far)

```
  struct Actor {
  ..........
}
```

The Actor struct contains all the variables needed for an Actor. An actor is basically a game object and can be either a wall, player or an enemy.
