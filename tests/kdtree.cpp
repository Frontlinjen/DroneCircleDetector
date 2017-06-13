#include <gtest/gtest.h>
#include "ring_detector/kdtree.h"

void printTree(const kdNode* root){
  if(root->type == kdNode::LEAF){
    std::cout << "Leaf(" << root->leaf.p.x << ", " << root->leaf.p.y << ")\n";
  }else{
	std::cout << "Node(" << root->branch.location << ")\n";
    if(root->branch.left != NULL){
      std::cout << "Visiting left branch ";
      printTree(root->branch.left);
    }
    if(root->branch.right != NULL){
      std::cout << "Visiting right branch ";
      printTree(root->branch.right);
    }
  }
};

TEST(KdTree, canBuildTree){
  std::vector<kdPoint> points{{1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 10}, {11, 12}, {13, 14}, {15, 16}, {17, 18}, {19, 20}, {21, 22}, {23, 24}, {25, 26}, {27, 28}, {29, 30}};
  kdTree tree;
  tree.buildTree(points);
  const kdNode * root = tree.getRoot();
  printTree(root);
}


TEST(KdTree, canSearchTree){
  std::vector<kdPoint> points{{1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 10}, {11, 12}, {13, 14}, {15, 16}, {17, 18}, {19, 20}, {21, 22}, {23, 24}, {25, 26}, {27, 28}, {29, 30}};
  kdTree tree;
  tree.buildTree(points);

  Rect r;
  r.center = kdPoint{15, 19};
  r.height = 5;
  r.width = 2;
  std::vector<kdPoint> result = tree.InRange(r);
  EXPECT_EQ(3, result.size());
}

main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
