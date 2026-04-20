BRANCH="$(git rev-parse --abbrev-ref HEAD)"
GIT_HASH="$(git rev-parse --short HEAD)"
BUILD_TIME="$(date -R)"
BUILD_TYPE="$(basename "$PWD")"

cat > ../Core/Inc/git-hash.h <<EOF
/*
 *  git-hash.h
 *
 *  Auto-generated during the STM32 pre-build steps
 *  Last generated: $BUILD_TIME
 */

#ifndef INC_GIT_HASH_H_
#define INC_GIT_HASH_H_

#define GIT_HASH        "$GIT_HASH"
#define GIT_BRANCH      "$BRANCH"
#define BUILD_TIME      "$BUILD_TIME"
#define BUILD_TYPE      "$BUILD_TYPE"

#endif /* INC_GIT_HASH_H_ */
EOF