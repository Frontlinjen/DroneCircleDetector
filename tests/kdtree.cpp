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
  std::vector<kdLeaf> points {{{1, 2}, NULL}, {{3, 4}, NULL}, {{5, 6}, NULL}, {{7, 8}, NULL}, {{9, 10}, NULL}, {{11, 12}, NULL}, {{13, 14}, NULL}, {{15, 16}, NULL}, {{17, 18}, NULL}, {{19, 20} ,NULL}, {{21, 22}, NULL}, {{23, 24}, NULL}, {{25, 26}, NULL}, {{27, 28}, NULL}, {{29, 30}, NULL}};
  kdTree tree;
  tree.buildTree(points);
  const kdNode * root = tree.getRoot();
  printTree(root);
}


TEST(KdTree, canSearchTree){
  std::vector<kdLeaf> points {{{1, 2}, NULL}, {{3, 4}, NULL}, {{5, 6}, NULL}, {{7, 8}, NULL}, {{9, 10}, NULL}, {{11, 12}, NULL}, {{13, 14}, NULL}, {{15, 16}, NULL}, {{17, 18}, NULL}, {{19, 20} ,NULL}, {{21, 22}, NULL}, {{23, 24}, NULL}, {{25, 26}, NULL}, {{27, 28}, NULL}, {{29, 30}, NULL}};
  kdTree tree;
  tree.buildTree(points);

  Rect r;
  r.center = kdPoint{15, 19};
  r.height = 5;
  r.width = 2;
  std::vector<kdLeaf> result = tree.InRange(r);
  EXPECT_EQ(3, result.size());
}

main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
